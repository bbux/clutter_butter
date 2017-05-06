/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief odom tracker node for serving up odom info
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
#include "odom_tracker.h"

OdomTracker::OdomTracker(ros::NodeHandle nh) {
  n = nh;
  // Register our services with the master.
  getOdomService = n.advertiseService("get_odom", &OdomTracker::getOdom, this);
  odomSubscriber = n.subscribe("odom", 10, &OdomTracker::handleOdom, this);
}

OdomTracker::~OdomTracker() {
  // nothing to do
}

void OdomTracker::spin() {
  ROS_INFO_STREAM("Starting Odom Tracker ...");
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool OdomTracker::getOdom(clutter_butter::GetOdom::Request &req, clutter_butter::GetOdom::Response &resp) {
  resp.pose = location;
  return true;
}

void OdomTracker::handleOdom(nav_msgs::Odometry odom) {
  ROS_DEBUG_STREAM("New Odom At: \n" << odom.pose.pose);
  location.orientation.w = odom.pose.pose.orientation.w;
  location.orientation.x = odom.pose.pose.orientation.x;
  location.orientation.y = odom.pose.pose.orientation.y;
  location.orientation.z = odom.pose.pose.orientation.z;

  location.position.x = odom.pose.pose.position.x;
  location.position.y = odom.pose.pose.position.y;
  location.position.z = odom.pose.pose.position.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_tracker");
  ros::NodeHandle n;
  OdomTracker odom_tracker(n);
  odom_tracker.spin();
  return 0;
}
