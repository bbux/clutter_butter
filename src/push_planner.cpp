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
#include "push_planner.h"

PushPlanner::PushPlanner(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  addTargetService = n.advertiseService("add_target", &PushPlanner::addTarget,
                                        this);
  getPushPlanService = n.advertiseService("get_push_plan",
                                          &PushPlanner::getPushPlan, this);
}

PushPlanner::~PushPlanner() {
  // nothing to do
}

void PushPlanner::spin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // any new targets to create plans for?


    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool PushPlanner::addTarget(clutter_butter::NewTargetRequest &req,
                            clutter_butter::NewTargetResponse &resp) {

  //TODO: implement
  return false;
}

bool PushPlanner::getPushPlan(clutter_butter::GetPushPlanRequest &req,
                              clutter_butter::GetPushPlanResponse &resp) {
  //TODO: implement
  return false;
}

int main(int argc, char **argv) {
  // basic initialization, then let push_planner class do the rest
  ros::init(argc, argv, "push_planner");
  ros::NodeHandle n;
  PushPlanner push_planner(n);
  push_planner.spin();
  return 0;
}
