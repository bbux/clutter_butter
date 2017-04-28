/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief push_planner node header definitions
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

#ifndef PUSH_PLANNER_H_
#define PUSH_PLANNER_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include "clutter_butter/Target.h"
#include "clutter_butter/NewTarget.h"
#include "clutter_butter/GetPushPlan.h"

/**
 * Class for creating push plans for Targets to push them to the coordinates of the
 * configured jail
 */
class PushPlanner {
 private:
  ros::NodeHandle n;
  // subscriber for Target push updates
  ros::Subscriber sub;
  // location of the Jail relative to the world frame
  geometry_msgs::Point jail;
  ros::ServiceServer addTargetService;
  ros::ServiceServer getPushPlanService;

 public:
  /**
   * Construct and initialize the push_planner node
   * @param nh the valid node handle for this node
   */
  explicit PushPlanner(ros::NodeHandle nh);
  /**
   * Starts the node loop to listen for new targets to create plans for
   */
  void spin();
  /**
   * Service for adding a new target
   */
  bool addTarget(clutter_butter::NewTargetRequest &req,
                 clutter_butter::NewTargetResponse &resp);
  /**
   * Service for getting a push plan, if any are available
   */
  bool getPushPlan(clutter_butter::GetPushPlanRequest &req,
                   clutter_butter::GetPushPlanResponse &resp);
  /**
   * Destructor
   */
  virtual ~PushPlanner();
};

#endif  // PUSH_PLANNER_H_
