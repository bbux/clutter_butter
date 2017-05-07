/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief push executor node header definitions
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

#ifndef PUSH_EXECUTOR_H_
#define PUSH_EXECUTOR_H_

#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "clutter_butter/Target.h"
#include "clutter_butter/GetPushPlan.h"
#include "clutter_butter/SetPushExecutorState.h"
#include "clutter_butter/GoTo.h"
#include "clutter_butter/Orient.h"
#include "clutter_butter/GetOdom.h"


/**
 * Class for executing PushPlans
 */
class PushExecutor {
 private:
  ros::NodeHandle n;
  // services
  ros::ServiceServer setStateService;
  ros::ServiceServer goToService;
  ros::ServiceServer orientService;
  // service clients
  ros::ServiceClient getPushPlanClient;
  ros::ServiceClient getOdomClient;
  // publisher for velocity twist messages
  ros::Publisher velocityPub;
  // state
  int mode = clutter_butter::SetPushExecutorState::Request::STOPPED;

  // internal methods
  /**
   * Executes the given plan
   * @param plan to execute
   */
  void executePlan(clutter_butter::PushPlan plan);
  /**
   * Moves the robot to the location specified
   * @param point to move to
   * @angle to orient to
   * @return if move was successful
   */
  bool goTo(geometry_msgs::Point goal, double angle);

  /**
   * rotates the robot to the desire angle
   * @param angle in degrees to orient to
   */
  void setOrientation(double angle);
  /**
   *  sends velocity message to rotate desired degrees
   *  @param angle to rotate
   *  @param speed to rotate by
   *  @return if able to do the rotation
   */
  bool rotateNDegrees(double angle, double speed);
  /**
   * sends velocity message to halt the robot in place
   */
  void stop();
  /**
   * sends velocity command to move forward by the provided increment
   * @param increment
   */
  void forward(float increment);
  /**
   * get current odom
   */
  geometry_msgs::Pose getOdom();

 public:
  /**
   * Construct and initialize the push_planner node
   * @param nh the valid node handle for this node
   */
  explicit PushExecutor(ros::NodeHandle nh);
  /**
   * Destructor
   */
  virtual ~PushExecutor();
  /**
   * Starts the node loop to listen for activation
   */
  void spin();

  /**
   * Service for setting the push executor state
   */
  bool setState(clutter_butter::SetPushExecutorStateRequest &req, clutter_butter::SetPushExecutorStateResponse &resp);

  /**
   * Service for go to service (debug mode only)
   */
  bool goToServiceHandler(clutter_butter::GoToRequest &req, clutter_butter::GoToResponse &resp);

  /**
   * Service for orient service (debug mode only)
   */
  bool orientServiceHandler(clutter_butter::OrientRequest &req, clutter_butter::OrientResponse &resp);
};

#endif  // PUSH_EXECUTOR_H_
