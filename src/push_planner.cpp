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
#include "ros/ros.h"
#include <math.h>
#include <exception>
#include <vector>
#include "clutter_butter/push_planner.h"

PushPlanner::PushPlanner(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  addTargetService = n.advertiseService("add_target", &PushPlanner::addTarget, this);
  updateTargetService = n.advertiseService("update_target",  &PushPlanner::updateTarget, this);
  getPushPlanService = n.advertiseService("get_push_plan", &PushPlanner::getPushPlan, this);
  clearAllService = n.advertiseService("clear_push_planner", &PushPlanner::clearAll, this);
  // initialize jail
  n.param<double>("jail_x", jail.x, 2.0);
  n.param<double>("jail_y", jail.y, 0.0);
  n.param<double>("jail_z", jail.z, 0.0);

  // get initial offset
  n.param<double>("offset", offset, 0.25);
  // set reasonable min dist
  double offset_dist = sqrt(pow(offset, 2) + pow(offset, 2));
  if (offset_dist < 2) {
    minimumDistance = offset_dist;
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
      clutter_butter::Target end = targets.back();
      targets.pop_back();
      clutter_butter::PushPlan plan = createPushPlan(end);
      ROS_INFO_STREAM("Created plan for target with id: " << plan.target.id);
      plans.push_back(plan);
    }

    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool PushPlanner::addTarget(clutter_butter::NewTargetRequest &req, clutter_butter::NewTargetResponse &resp) {
  ROS_INFO_STREAM("Request to add target at centroid: (" << req.centroid.x << ", " << req.centroid.y << ")");
  // is it already in jail?
  double dist = distance(req.centroid, jail);
  if (dist < minimumDistance) {
    ROS_WARN_STREAM("Target at (" << req.centroid.x << ", " << req.centroid.y << ") "
                    << dist << " from jail, min dist is " << minimumDistance);
    return false;
  }
  // is it already known?
  int targetId = targetExists(req.centroid);
  if (targetId != -1) {
    ROS_DEBUG_STREAM("Target at centroid: (" << req.centroid.x << ", " << req.centroid.y << ") already exists");
    clutter_butter::Target target = getTarget(targetId);
    resp.target = target;
  } else {
    ROS_DEBUG_STREAM("Creating new target...");
    clutter_butter::Target target = createTarget(req.centroid);
    resp.target = target;
  }
  return true;
}

bool PushPlanner::updateTarget(clutter_butter::UpdateTargetRequest &req, clutter_butter::UpdateTargetResponse &resp) {
  if (req.action == clutter_butter::UpdateTarget::Request::JAILED) {
    for (std::vector<clutter_butter::PushPlan>::iterator it = plans.begin(); it != plans.end(); ++it) {
      clutter_butter::PushPlan &plan = *it;
      if (plan.target.id == req.target.id) {
        ROS_INFO_STREAM("Setting Target with ID: " << plan.target.id << " To JAILED");
        plan.jailed = true;
        break;
      }
    }
  } else {
    // they moved it
    // TODO(bbuxton): figure this out
  }
  (void)resp;  // Suppress unused warning
  return true;
}

clutter_butter::PushPlan PushPlanner::getPushPlanForTarget(int id) {
  for (clutter_butter::PushPlan plan : plans) {
    if (plan.target.id == id) {
      return plan;
    }
  }
  ROS_ERROR_STREAM("No target with id: " << id << " exists!!");
  // cant find it
  throw std::domain_error("invalid target id");
}

bool PushPlanner::getPushPlan(clutter_butter::GetPushPlanRequest &req, clutter_butter::GetPushPlanResponse &resp) {
  (void)req;  // Suppress unused warning
  if (plans.size() > 0) {
    int id = plans.front().target.id;
    double shortestDistance = 1000000;
    for (clutter_butter::PushPlan plan : plans) {
      ROS_INFO_STREAM("Plan: " << plan.target.id << ", Jailed: " << (plan.jailed == true));
      // skip jailed plans
      if (plan.jailed == true) {
        continue;
      }
      double dist = distance(plan.target.centroid, jail);
      ROS_DEBUG_STREAM("Distance for " << plan.target.id << " is " << dist);
      if (dist < shortestDistance) {
        ROS_DEBUG_STREAM("New shortest distance: " << plan.target.id);
        id = plan.target.id;
        shortestDistance = dist;
      }
    }
    resp.plan = getPushPlanForTarget(id);
    ROS_DEBUG_STREAM("Found Push Plan with target id: " << resp.plan.target.id);
    resp.isvalid = true;
    return true;
  } else {
    ROS_INFO_STREAM("Requested Push Plan, but none exists...");
    return false;
  }
}

bool PushPlanner::clearAll(clutter_butter::ClearAllRequest &req, clutter_butter::ClearAllResponse &resp) {
  (void)req;  // Suppress unused warning
  (void)resp;  // Suppress unused warning
  targets.clear();
  plans.clear();
  currentTargetId = 0;
  return true;
}

int PushPlanner::targetExists(geometry_msgs::Point centroid) {
  for (clutter_butter::PushPlan plan : plans) {
    // better way to do this?
    double dist = distance(plan.target.centroid, centroid);
    ROS_DEBUG_STREAM("Distance to target: " << dist);
    if (dist <= minimumDistance) {
      ROS_DEBUG_STREAM("Target with id" << plan.target.id << " less that minimum distance of " << minimumDistance);
      return plan.target.id;
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
  plan.jailed = false;
  geometry_msgs::Pose start;
  geometry_msgs::Point startpos;
  geometry_msgs::Quaternion startOrientation;

  // easy cases, where target is perfectly aligned
  if (target.centroid.x == jail.x) {
    ROS_DEBUG_STREAM("Target Aligned Along X Axis");
    startpos.x = target.centroid.x;
    // shift by appropriate offset
    if (target.centroid.y < jail.y) {
      startpos.y = target.centroid.y - offset;
      setOrientation(startOrientation, 90);
    } else {
      startpos.y = target.centroid.y + offset;
      setOrientation(startOrientation, 270);
    }
  } else if (target.centroid.y == jail.y) {
    ROS_DEBUG_STREAM("Target Aligned Along Y Axis");
    startpos.y = target.centroid.y;
    // shift by appropriate offset
    if (target.centroid.x < jail.x) {
      startpos.x = target.centroid.x - offset;
      setOrientation(startOrientation, 0);
    } else {
      startpos.x = target.centroid.x + offset;
      setOrientation(startOrientation, 180);
    }
  } else {
    // harder case
    // angle from target to jail centroid
    double angle = M_PI * atan2(jail.y - target.centroid.y, jail.x - target.centroid.x) / 180.0;
    setOrientation(startOrientation, 180);
    ROS_DEBUG_STREAM("Target at (" << target.centroid.x << ", " << target.centroid.y << ")");
    ROS_DEBUG_STREAM("Angle from Jail: " << angle);
  }
  start.position = startpos;
  start.orientation = startOrientation;

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

void PushPlanner::setOrientation(geometry_msgs::Quaternion &q, double zDegrees) {
  double zRadians = M_PI * zDegrees / 180.0;
  double t0 = std::cos(zRadians * 0.5);
  double t1 = std::sin(zRadians * 0.5);
  double t2 = std::cos(0 * 0.5);
  double t3 = std::sin(0 * 0.5);
  double t4 = std::cos(0 * 0.5);
  double t5 = std::sin(0 * 0.5);

  q.w = t0 * t2 * t4 + t1 * t3 * t5;
  q.z = t1 * t2 * t4 - t0 * t3 * t5;
  // these should be zero
  q.x = t0 * t3 * t4 - t1 * t2 * t5;
  q.y = t0 * t2 * t5 + t1 * t3 * t4;
}

int main(int argc, char **argv) {
// basic initialization, then let push_planner class do the rest
ros::init(argc, argv, "push_planner");
ros::NodeHandle n;
PushPlanner push_planner(n);
push_planner.spin();
return 0;
}
