/**
 * @file
 * @author Brian Buxton <bbux10@gmail.com>
 * @brief unit test for TODO: Fill in brief
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
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include "clutter_butter/Target.h"
#include "clutter_butter/NewTarget.h"
#include "clutter_butter/GetPushPlan.h"

std::shared_ptr<ros::NodeHandle> n;

TEST(ServiceTests, servicesExist) {
  ros::ServiceClient addTargetClient = n->serviceClient < clutter_butter::NewTarget > ("add_target");
  ros::ServiceClient getPushPlanClient = n->serviceClient < clutter_butter::GetPushPlan > ("get_push_plan");

  bool addTargetExists(addTargetClient.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(addTargetExists);

  bool getPushPlanExists(getPushPlanClient.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(getPushPlanExists);
}

TEST(ServiceTests, addSingleTarget) {
  ros::ServiceClient client = n->serviceClient < clutter_butter::NewTarget > ("add_target");

  clutter_butter::NewTarget srv;
  geometry_msgs::Point centroid;
  centroid.x = 1.0;
  centroid.y = 1.0;
  centroid.z = 0.0;
  srv.request.centroid = centroid;

  client.call(srv);

  // verify that the returned Target has an ID and the centroids match
  EXPECT_GE(0, srv.response.target.id);
  EXPECT_EQ(1.0, srv.response.target.centroid.x);
  EXPECT_EQ(1.0, srv.response.target.centroid.y);
  EXPECT_EQ(0.0, srv.response.target.centroid.z);
}

TEST(ServiceTests, targetIsAlreadyInJail) {
  ros::ServiceClient add_target = n->serviceClient < clutter_butter::NewTarget > ("add_target");
  ros::ServiceClient get_plan = n->serviceClient < clutter_butter::NewTarget > ("get_push_plan");

  clutter_butter::NewTarget srv1;
  geometry_msgs::Point centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;
  srv1.request.centroid = centroid;

  add_target.call(srv1);

  clutter_butter::GetPushPlan srv2;
  get_plan.call(srv2);

  // since already in jail, start is goal is jail
  EXPECT_EQ(0.0, srv2.response.plan.start.position.x);
  EXPECT_EQ(0.0, srv2.response.plan.start.position.y);
  EXPECT_EQ(0.0, srv2.response.plan.start.position.z);
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.x);
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.y);
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.z);
}

TEST(ServiceTests, straitLineFromTargetToJail) {
  ros::ServiceClient add_target = n->serviceClient < clutter_butter::NewTarget > ("add_target");
  ros::ServiceClient get_plan = n->serviceClient < clutter_butter::NewTarget > ("get_push_plan");

  clutter_butter::NewTarget srv1;
  geometry_msgs::Point centroid;
  // offset 4 to the right
  centroid.x = 4.0;
  centroid.y = 0.0;
  centroid.z = 0.0;
  srv1.request.centroid = centroid;
  double offset = 1.0;

  add_target.call(srv1);

  clutter_butter::GetPushPlan srv2;
  get_plan.call(srv2);

  // expect start to be offset by a little from center of target
  EXPECT_EQ(centroid.x + offset, srv2.response.plan.start.position.x);
  EXPECT_EQ(centroid.y, srv2.response.plan.start.position.y);
  EXPECT_EQ(centroid.z, srv2.response.plan.start.position.z);
  // destinatioin is jail
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.x);
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.y);
  EXPECT_EQ(0.0, srv2.response.plan.goal.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "push_planner_tests");
  n.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
