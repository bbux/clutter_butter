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

PushExecutor::PushExecutor(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  setStateService = n.advertiseService("set_executor_state", &PushExecutor::setState, this);
  // create client to get push plans
  getPushPlanClient = n.serviceClient<clutter_butter::GetPushPlan>("get_push_plan");
  n.param<bool>("start_active", active, false);
  ROS_INFO_STREAM("Starting in Active Mode: " << active);
}

PushExecutor::~PushExecutor() {
  // nothing to do
}

void PushExecutor::spin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // any plans to execute
    if (active) {
      ROS_INFO_STREAM("Were active, time to push stuff...");
      clutter_butter::GetPushPlan getPushPlanService;
      bool hasPlan = getPushPlanClient.call(getPushPlanService);
      if (hasPlan) {
        executePlan(getPushPlanService.response.plan);
      }
    }

    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool PushExecutor::setState(clutter_butter::SetPushExecutorStateRequest &req,
                            clutter_butter::SetPushExecutorStateResponse &resp) {
  active = (req.state == clutter_butter::SetPushExecutorStateRequest::PUSHING);
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

bool PushExecutor::goTo(geometry_msgs::Pose pose) {
  ROS_INFO_STREAM("Attempting to move to (" << pose.position.x << ", " << pose.position.y << ")");
  // TODO: Move the bot!
  return true;
}

int main(int argc, char **argv) {
  // basic initialization, then let push_executor class do the rest
  ros::init(argc, argv, "push_executor");
  ros::NodeHandle n;
  PushExecutor push_executor(n);
  push_executor.spin();
  return 0;
}
