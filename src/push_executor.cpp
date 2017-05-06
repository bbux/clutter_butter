/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief push_executor node for executing push plans
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
#include "push_executor.h"
#include "clutter_butter/SetPushExecutorState.h"

geometry_msgs::Twist zero_twist() {
  geometry_msgs::Twist vel;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  return vel;
}

double angleDifference(geometry_msgs::Point start, geometry_msgs::Point goal) {
  return M_PI * atan2(goal.y - start.y, goal.x - start.x) / 180.0;
}

double calculateDistance(geometry_msgs::Point start, geometry_msgs::Point goal) {
  return sqrt(pow(goal.y - start.y, 2) + pow(goal.x - start.x, 2));
}

double quaternionToZAngle(const geometry_msgs::Quaternion &q) {
  double ysqr = q.y * q.y;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w * q.z + q.x * q.y);
  double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
  double yaw = std::atan2(t3, t4);

  return yaw * 180.0 / M_PI;
}

void setOrientationFromAngle(geometry_msgs::Quaternion &q, double zDegrees) {
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

PushExecutor::PushExecutor(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  setStateService = n.advertiseService("set_executor_state", &PushExecutor::setState, this);
  goToService = n.advertiseService("executor_go_to", &PushExecutor::goToServiceHandler, this);
  orientService = n.advertiseService("executor_set_orientation", &PushExecutor::orientServiceHandler, this);
  // create client to get push plans
  getPushPlanClient = n.serviceClient<clutter_butter::GetPushPlan>("get_push_plan");
  getOdomClient = n.serviceClient<clutter_butter::GetOdom>("get_odom");
  velocityPub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  int defaultMode = clutter_butter::SetPushExecutorState::Request::DEBUG;
  n.param<int>("start_mode", mode, defaultMode);
  ROS_INFO_STREAM("Starting in Mode: " << mode);
}

PushExecutor::~PushExecutor() {
  // nothing to do
}

void PushExecutor::spin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // any plans to execute
    if (mode == clutter_butter::SetPushExecutorState::Request::PUSHING) {
      ROS_DEBUG_STREAM("Were active, time to push stuff...");
      clutter_butter::GetPushPlan getPushPlanService;
      bool hasPlan = getPushPlanClient.call(getPushPlanService);
      if (hasPlan) {
        executePlan(getPushPlanService.response.plan);
      }
    } else if (mode == 2 ) { // debug
      ROS_DEBUG_STREAM("Were now in debug mode...");
    }
    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool PushExecutor::setState(clutter_butter::SetPushExecutorStateRequest &req,
                            clutter_butter::SetPushExecutorStateResponse &resp) {
  mode = req.state;
  ROS_INFO_STREAM("Set state to: " << req.state);
  // is this useful?
  resp.result = clutter_butter::SetPushExecutorStateResponse::OK;
  return true;
}

bool PushExecutor::goToServiceHandler(clutter_butter::GoToRequest &req, clutter_butter::GoToResponse &resp) {
  ROS_INFO_STREAM("Go To Request: " << req);
  int distance = goTo(req.goal, req.angleDegrees);
  resp.distance = distance;
  return (distance > 0);
}

bool PushExecutor::orientServiceHandler(clutter_butter::OrientRequest &req, clutter_butter::OrientResponse &resp) {
  setOrientation(quaternionToZAngle(req.orientation));
  return true;
}

void PushExecutor::executePlan(clutter_butter::PushPlan plan) {
  ROS_INFO_STREAM("Executing plan for target with id: " << plan.target.id << "...");
  // go to start position, avoiding obstacles along the way
  goTo(plan.start.position, quaternionToZAngle(plan.start.orientation));
  // push target keeping track of centroid, and sending this to target update service

  // if off track, abandon this plan and request a new one
}

bool PushExecutor::goTo(geometry_msgs::Point goal, double desiredAngle) {
  ROS_INFO_STREAM("Attempting to move to (" << goal.x << ", " << goal.y << "), angle: " << desiredAngle);
  geometry_msgs::Pose location = getOdom();
  // what is the angle difference between where we are now and where we want to go?
  double angleToGoal = angleDifference(location.position, goal);
  // how far are we from the goal
  double distance = calculateDistance(location.position, goal);
  ROS_INFO_STREAM("Goal is distance: " << distance << " at angle " << angleToGoal);

  // first align our orientation to point to the goal
  setOrientation(angleToGoal);
  int count = 0;
  double speed = 0.5;
  while(distance > 0.1) {
    distance = calculateDistance(getOdom().position, goal);
    ROS_INFO_STREAM("Current distance: " << distance);
    count++;
    if (count > 100) {
      ROS_WARN_STREAM("Don't seem to be making progress towards goal after " << count << " iterations");
      return distance;
    }
  }
  ROS_INFO_STREAM("We made it to the goal! Setting orientation to: " << desiredAngle);
  setOrientation(desiredAngle);

  location = getOdom();
  return calculateDistance(location.position, goal);
}

void PushExecutor::setOrientation(int desiredAngle) {
  ROS_INFO_STREAM("Attempting to orient to " << desiredAngle << " degrees");

  geometry_msgs::Pose location = getOdom();
  double currentAngle = quaternionToZAngle(location.orientation);

  double delta = desiredAngle - currentAngle;

  while (delta > 0.01) {
    ROS_INFO_STREAM("Angle delta: " << delta << " degrees");
    if (fabs(delta) > 20) {
      rotateNDegrees(delta, 20);
    // slow down a little
    } else if (fabs(delta) < 20 && fabs(delta) > 5) {
      rotateNDegrees(delta, 5);
    // slow down a lot
    } else {
      rotateNDegrees(delta, 1);
    }
    currentAngle = quaternionToZAngle(getOdom().orientation);
    delta = desiredAngle - currentAngle;
    // TODO: Delete this!!!!
    ros::Duration(1).sleep();
  }
}

void PushExecutor::rotateNDegrees(int angle, double speed) {
  geometry_msgs::Twist vel = zero_twist();

  double angular_speed = speed * 2 * M_PI / 360.0;
  double relative_angle = angle * 2 * M_PI / 360.0;
  vel.angular.z = angular_speed;
  ROS_DEBUG_STREAM("Attempting to rotate " << angle << " degrees with speed " << angular_speed);

  double current_angle = 0;
  int t0 = ros::Time::now().sec;

  while (current_angle < relative_angle) {
    velocityPub.publish(vel);
    int t1 = ros::Time::now().sec;
    current_angle = angular_speed * (t1 - t0);
  }
}

void PushExecutor::forward(float increment) {
  ROS_DEBUG_STREAM("moving forward by " << increment);

  geometry_msgs::Twist vel = zero_twist();
  vel.linear.x = increment;
  velocityPub.publish(vel);
}

void PushExecutor::stop() {
  geometry_msgs::Twist vel = zero_twist();
  velocityPub.publish(vel);
}

geometry_msgs::Pose PushExecutor::getOdom() {
  clutter_butter::GetOdom srv;
  getOdomClient.call(srv);
  return srv.response.pose;
}

int main(int argc, char **argv) {
  // basic initialization, then let push_executor class do the rest
  ros::init(argc, argv, "push_executor");
  ros::NodeHandle n;
  PushExecutor push_executor(n);
  push_executor.spin();
  return 0;
}
