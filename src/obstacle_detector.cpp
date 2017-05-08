/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief node for identifying obstacles
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
#include "obstacle_detector.h"
#include <vector>

float MINDIST = 0.3;
float INCREMENT = 0.5;
int MAXSTEPS = 10;
double PI = 3.1415926535897;

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

/**
 * Checks to see if avg values in ranges is below the provided threshold
 * or if the min seen is half the value of the min_dist
 */
bool too_close(std::vector<float> ranges, float min_dist) {
  double total = 0;
  // store min we have seen
  float min = 1000000.0;
  // calculate average and keep track of current min
  for (float val : ranges) {
    if (val < min) {
      min = val;
    }
    total += val;
  }
  double avg = total / ranges.size();
  // avg is less than min or min seen is half that
  return avg < min_dist || min < (min_dist/2);
}


ObstacleDetector::ObstacleDetector(ros::NodeHandle nh) {
  n = nh;
  pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  setStateService = n.advertiseService("set_obstacle_detector_state", &ObstacleDetector::setState, this);
  ros::spin();
}

ObstacleDetector::~ObstacleDetector() {
  // nothing to do
}

bool ObstacleDetector::setState(clutter_butter::SetObstacleDetectorStateRequest &req,
              clutter_butter::SetObstacleDetectorStateResponse &resp) {
  enabled = (req.state == clutter_butter::SetObstacleDetectorState::Request::ACTIVE);
  return true;
}

void ObstacleDetector::process_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
  if (enabled && too_close(scan->ranges, MINDIST)) {
    ROS_DEBUG_STREAM("found an obstacle stopping and redirecting...");
    stop();
    rotate_n_degrees(5);
    stop();
  }
}


void ObstacleDetector::rotate_n_degrees(int angle) {
  geometry_msgs::Twist vel = zero_twist();

  // arbitrarily picked speed
  double angular_speed = 50 * 2 * PI / 360.0;
  double relative_angle = angle * 2 * PI / 360.0;
  vel.angular.z = angular_speed;
  ROS_DEBUG_STREAM("Attempting to rotate " << angle << " degrees with speed " << angular_speed);

  double current_angle = 0;
  int t0 = ros::Time::now().sec;

  while (current_angle < relative_angle) {
    pub.publish(vel);
    int t1 = ros::Time::now().sec;
    current_angle = angular_speed * (t1 - t0);
  }
}

void ObstacleDetector::stop() {
  geometry_msgs::Twist vel = zero_twist();
  pub.publish(vel);
}


int main(int argc, char **argv) {
  // basic initialization, then let obstacle_detector class do the rest
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle n;
  ObstacleDetector obstacle_detector(n);
  return 0;
}
