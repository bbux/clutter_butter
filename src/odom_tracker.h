/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief keeps track of odometry and serves it up with a service call
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

#ifndef ODOM_TRACKER_H_
#define ODOM_TRACKER_H_

#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "clutter_butter/GetOdom.h"
#include "clutter_butter/GetOdomPretty.h"

/**
 * Class for tracking odometry and serving it up through a service
 */
class OdomTracker {
 private:
  ros::NodeHandle n;
  ros::ServiceServer getOdomService;
  ros::ServiceServer getOdomPrettyService;
  // subscriber for odometry, where are we now?
  ros::Subscriber odomSubscriber;
  // current location
  geometry_msgs::Pose location;

  /**
   * handler for odom subscriber
   */
  void handleOdom(nav_msgs::Odometry odom);

 public:
  /**
   * Construct and initialize
   * @param nh the valid node handle for this node
   */
  explicit OdomTracker(ros::NodeHandle nh);
  /**
   * Destructor
   */
  virtual ~OdomTracker();
  /**
   * Starts the node loop
   */
  void spin();
  /**
   * Service for get current odom
   */
  bool getOdom(clutter_butter::GetOdom::Request &req, clutter_butter::GetOdom::Response &resp);

  /**
   * Service for get current odom in more readable format
   */
  bool getOdomPretty(clutter_butter::GetOdomPretty::Request &req, clutter_butter::GetOdomPretty::Response &resp);
};

#endif  // ODOM_TRACKER_H_
