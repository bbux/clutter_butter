/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief obstacle detector node header definitions
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

#ifndef SRC_OBSTACLE_DETECTOR_H_
#define SRC_OBSTACLE_DETECTOR_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "clutter_butter/SetObstacleDetectorState.h"

/**
 * Class for performing obstacle avoidance
 */
class ObstacleDetector {
 private:
  ros::NodeHandle n;
  // publisher for velocity twist messages
  ros::Publisher pub;
  // set state service
  ros::ServiceServer setStateService;
  // mode
  bool enabled = false;
  /**
   * random walk algorithm happens here.  Uses the laser scan data to determine
   * proximity to any obstacles and to avoid them by modifying the velocity
   */
  void process_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
  /**
   * sends velocity messages to rotate the robot angle degrees
   * @param angle in degrees to rotate
   */
  void rotate_n_degrees(int angle);
  /**
   * sends velocity message to halt the robot in place
   */
  void stop();

 public:
  /**
   * Construct and initialize the node
   * @param nh the valid node handle for this node
   */
  explicit ObstacleDetector(ros::NodeHandle nh);
  /**
   * Starts the node loop to listen for laser scan data and to send updated
   * velocity commands to perform the obstacle avoidance
   */
  void spin();
  /**
   * Destructor
   */
  virtual ~ObstacleDetector();
  /**
   * Service for setting the obstacle detector state
   */
  bool setState(clutter_butter::SetObstacleDetectorStateRequest &req,
                clutter_butter::SetObstacleDetectorStateResponse &resp);
};

#endif  // SRC_OBSTACLE_DETECTOR_H_
