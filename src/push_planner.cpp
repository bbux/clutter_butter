/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief push_planner node for planning paths for pushing targets
 * @copyright BSD License
 * Copyright (c) 2017 Brian Buxton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
#include <exception>
#include "ros/ros.h"
#include "push_planner.h"

PushPlanner::PushPlanner(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  addTargetService = n.advertiseService("add_target", &PushPlanner::addTarget, this);
  getPushPlanService = n.advertiseService("get_push_plan", &PushPlanner::getPushPlan, this);

  // initialize jail
  n.param<double>("jail_x", jail.x, 0.0);
  n.param<double>("jail_y", jail.y, 0.0);
  n.param<double>("jail_z", jail.z, 0.0);

  // get initial offset
  n.param<double>("offset", offset, 1.0);
  // set reasonable min dist
  double offset_dist = sqrt(pow(offset, 2) + pow(offset, 2));
  if (offset_dist < 2) {
    min_dist = offset_dist;
  }
  ROS_INFO_STREAM("Jail Coordinates: (" << jail.x << "," << jail.y << "," << jail.z << ")");
}

PushPlanner::~PushPlanner() {
  // nothing to do
}

void PushPlanner::spin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // any new targets to create plans for?
    if (targets.size() > 0) {
      clutter_butter::PushPlan plan = createPushPlan(targets.front());
      plans.push_back(plan);
    }

    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool PushPlanner::addTarget(clutter_butter::NewTargetRequest &req, clutter_butter::NewTargetResponse &resp) {
  int targetId = targetExists(req.centroid);
  if (targetId != -1) {
    ROS_INFO_STREAM("Target at centroid: (" << req.centroid.x << ", " << req.centroid.y << ") already exists");
    clutter_butter::Target target = getTarget(targetId);
    resp.target = target;
  } else {
    ROS_INFO_STREAM("Creating new target...");
    clutter_butter::Target target = createTarget(req.centroid);
    resp.target = target;
  }
  return true;
}

bool PushPlanner::getPushPlan(clutter_butter::GetPushPlanRequest &req, clutter_butter::GetPushPlanResponse &resp) {
  if (plans.size() > 0) {
    ROS_INFO_STREAM("Found Push Plan...");
    resp.plan = plans.front();
    return true;
  } else {
    ROS_INFO_STREAM("Requested Push Plan, but none exists...");
    return false;
  }
}

int PushPlanner::targetExists(geometry_msgs::Point centroid) {
  for (clutter_butter::Target target : targets) {
    // better way to do this?
    double dist = distance(target.centroid, centroid);
    ROS_INFO_STREAM("Distance to target: " << dist);
    if (dist <= min_dist) {
      return target.id;
    }
  }
  return -1;
}

clutter_butter::Target PushPlanner::getTarget(int targetId) {
  for (clutter_butter::Target t : targets) {
    if (t.id == targetId) {
      return t;
    }
  }
  ROS_ERROR_STREAM("No target with id: " << targetId << " exists!!");
  // cant find it
  throw std::domain_error("invalid target id");
}

clutter_butter::Target PushPlanner::createTarget(geometry_msgs::Point centroid) {
  clutter_butter::Target t;
  geometry_msgs::Point c;
  c.x = centroid.x;
  c.y = centroid.y;
  c.z = centroid.z;
  t.centroid = c;
  t.id = ++currentTargetId;

  targets.push_back(t);
  return t;
}

clutter_butter::PushPlan PushPlanner::createPushPlan(clutter_butter::Target target) {
  clutter_butter::PushPlan plan;


  geometry_msgs::Pose start;
  geometry_msgs::Point startpos;
  // easy cases, where target is perfectly aligned
  if (target.centroid.x == jail.x) {
    ROS_INFO_STREAM("Target Aligned Along X Axis");
    startpos.x = target.centroid.x;
    // shift by appropriate offset
    if (target.centroid.y < jail.y) {
      startpos.y = target.centroid.y - offset;
    } else {
      startpos.y = target.centroid.y + offset;
    }
  } else if (target.centroid.y == jail.y) {
    ROS_INFO_STREAM("Target Aligned Along Y Axis");
    startpos.y = target.centroid.y;
    // shift by appropriate offset
    if (target.centroid.x < jail.x) {
      startpos.x = target.centroid.x - offset;
    } else {
      startpos.x = target.centroid.x + offset;
    }
  } else {
    // harder case
    // angle from target to jail centroid
    double angle = atan2(jail.y - target.centroid.y, jail.x - target.centroid.x);
    ROS_INFO_STREAM("Target at (" << target.centroid.x << ", " << target.centroid.y << ")");
    ROS_INFO_STREAM("Angle from Jail: " << angle);
  }
  start.position = startpos;

  geometry_msgs::Pose goal;
  geometry_msgs::Point goalpos;
  goalpos.x = jail.x;
  goalpos.y = jail.y;
  goalpos.z = jail.z;
  goal.position = goalpos;

  plan.start = start;
  plan.goal = goal;
  plan.target = target;
  return plan;
}

double PushPlanner::distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y -p1.y, 2));
}

int main(int argc, char **argv) {
// basic initialization, then let push_planner class do the rest
ros::init(argc, argv, "push_planner");
ros::NodeHandle n;
PushPlanner push_planner(n);
push_planner.spin();
return 0;
}
