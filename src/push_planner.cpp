/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief push_planner node for roomba like walking and obstacle avoidance
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
#include <vector>
#include "ros/ros.h"
#include "push_planner.h"

float MINDIST = 0.3;
float INCREMENT = 0.5;
int MAXSTEPS = 10;
double PI = 3.1415926535897;

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

/**
 * get random degrees +- max amount
 */
int rand_degrees(int max) {
  // current time times large prime
  // mod this by twice the max
  // then shift to +- by subtracting the max
  return (ros::Time::now().sec * 12208373) % (2 * max) - max;
}

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

PushPlanner::PushPlanner(ros::NodeHandle nh) {
  n = nh;
  pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

PushPlanner::~PushPlanner() {
  // nothing to do
}

/*********************************************
 * ACTION HAPPENS HERE
 *********************************************/
void PushPlanner::process_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_DEBUG_STREAM("got a new scan message...");
  if (too_close(scan->ranges, MINDIST)) {
    ROS_DEBUG_STREAM("found an obstacle stopping and redirecting...");
    stop();
    rotate_n_degrees(5);
    stop();
  } else {
    forward(INCREMENT);
  }
}

void PushPlanner::walk() {
  ROS_DEBUG_STREAM("starting walk...");
  ros::Subscriber sub = n.subscribe("scan", 1000, &PushPlanner::process_scan, this);
  // start in opposite direction, makes demo more interesting
  rotate_n_degrees(180);
  // just sit here and wait, let process_scan do the work
  ros::spin();
}

void PushPlanner::stop() {
  geometry_msgs::Twist vel = zero_twist();
  pub.publish(vel);
}

void PushPlanner::rotate_n_degrees(int angle) {
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

void PushPlanner::forward(float increment) {
  ROS_DEBUG_STREAM("moving forward by: " << increment << " for time " << steps);

  geometry_msgs::Twist vel = zero_twist();
  vel.linear.x = increment;
  pub.publish(vel);

  // every once in a while change direction
  // to keep from marching off into space
  if (++steps > MAXSTEPS) {
    stop();
    steps = 0;
    rotate_n_degrees(rand_degrees(30));
    // forward again
    pub.publish(vel);
    stop();
  }
}

int main(int argc, char **argv) {
  // basic initialization, then let push_planner class do the rest
  ros::init(argc, argv, "push_planner");
  ros::NodeHandle n;
  PushPlanner push_planner(n);
  push_planner.walk();
  return 0;
}
