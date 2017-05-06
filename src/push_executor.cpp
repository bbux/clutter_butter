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

PushExecutor::PushExecutor(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  setStateService = n.advertiseService("set_executor_state", &PushExecutor::setState, this);
  // create client to get push plans
  getPushPlanClient = n.serviceClient<clutter_butter::GetPushPlan>("get_push_plan");
  velocityPub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  odomSubscriber = n.subscribe("odom", 10, &PushExecutor::handleOdom, this);
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

void PushExecutor::executePlan(clutter_butter::PushPlan plan) {
  ROS_INFO_STREAM("Executing plan for target with id: " << plan.target.id << "...");
  // go to start position, avoiding obstacles along the way
  goTo(plan.start);
  // push target keeping track of centroid, and sending this to target update service

  // if off track, abandon this plan and request a new one
}

bool PushExecutor::goTo(geometry_msgs::Pose goal) {
  ROS_INFO_STREAM("Attempting to move to (" << goal.position.x << ", " << goal.position.y << ")");
  // what is the angle difference between where we are now and where we want to go?
  double angle = angleDifference(location.position, goal.position);
  // how far are we from the goal
  double distance = calculateDistance(location.position, goal.position);
  // first align our orientation to point to the goal
  setOrientation(angle);
  int count = 0;
  double speed = 0.5;
  int t0 = ros::Time::now().sec;
  while(distance > 0.1) {
    forward(speed);
    int t1 = ros::Time::now().sec;
    distance = (t1 - t0) * speed;
    count++;
    if (count > 1000) {
      ROS_WARN_STREAM("Don't seem to be making progress towards goal after " << count << " iterations");
      return false;
    }
  }
  ROS_INFO_STREAM("We made it to the goal!");

  double desiredAngle = quaternionToZAngle(goal.orientation);
  setOrientation(desiredAngle);

  return true;
}

void PushExecutor::setOrientation(int desiredAngle) {
  geometry_msgs::Twist vel = zero_twist();

  // arbitrarily picked speed
  double angular_speed = 50 * 2 * M_PI / 360.0;
  vel.angular.z = angular_speed;
  ROS_INFO_STREAM("Attempting to orient to " << desiredAngle << " degrees");

  double currentAngle = quaternionToZAngle(location.orientation);
//  ROS_INFO_STREAM("Current angle " << currentAngle << " degrees");

  int t0 = ros::Time::now().sec;
  double delta = fabs(currentAngle - desiredAngle);

  while (delta > 0.01) {
    ROS_INFO_STREAM("Angle delta: " << delta << " degrees");
    velocityPub.publish(vel);
    int t1 = ros::Time::now().sec;
    currentAngle = angular_speed * (t1 - t0) * M_PI / 180.0;
    ROS_INFO_STREAM("Current angle " << currentAngle << " degrees");
    delta = fabs(currentAngle - desiredAngle);
    // slow down a little
    if (delta < 20) {
      angular_speed = 20 * 2 * M_PI / 360.0;
    // slow down a lot
    } else if (delta < 5) {
      angular_speed = 2 * 2 * M_PI / 360.0;
    }
    ros::Duration(10).sleep();
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

void PushExecutor::handleOdom(nav_msgs::Odometry odom) {
  location.orientation.w = odom.pose.pose.orientation.w;
  location.orientation.x = odom.pose.pose.orientation.x;
  location.orientation.y = odom.pose.pose.orientation.y;
  location.orientation.z = odom.pose.pose.orientation.z;

  double currentAngle = quaternionToZAngle(location.orientation);
  ROS_INFO_STREAM("Odom angle " << currentAngle << " degrees");

  location.position.x = odom.pose.pose.position.x;
  location.position.y = odom.pose.pose.position.y;
  location.position.z = odom.pose.pose.position.z;
}

int main(int argc, char **argv) {
  // basic initialization, then let push_executor class do the rest
  ros::init(argc, argv, "push_executor");
  ros::NodeHandle n;
  PushExecutor push_executor(n);
  push_executor.spin();
  return 0;
}
