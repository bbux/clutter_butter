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
#include "clutter_butter/ClearAll.h"

namespace {

// The fixture for testing class PushPlanner services.
class ServiceTest : public ::testing::Test {
 protected:
  ServiceTest() {
    n.reset(new ros::NodeHandle);
    add_target = n->serviceClient < clutter_butter::NewTarget > ("add_target");
    get_plan = n->serviceClient < clutter_butter::GetPushPlan > ("get_push_plan");
    // clear out all stored info
    ros::ServiceClient clear_all = n->serviceClient < clutter_butter::ClearAll > ("clear_push_planner");
    clutter_butter::ClearAll srv;
    clear_all.call(srv);
  }

  virtual ~ServiceTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // Objects declared here can be used by all tests in the test cases.
  std::shared_ptr<ros::NodeHandle> n;
  ros::ServiceClient add_target;
  ros::ServiceClient get_plan;

};
}


TEST_F(ServiceTest, servicesExist) {

  bool addTargetExists(add_target.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(addTargetExists);

  bool getPushPlanExists(get_plan.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(getPushPlanExists);
}

TEST_F(ServiceTest, addSingleTarget) {

  clutter_butter::NewTarget srv;
  geometry_msgs::Point centroid;
  centroid.x = 5.0;
  centroid.y = 5.0;
  centroid.z = 0.0;
  srv.request.centroid = centroid;

  add_target.call(srv);

  // verify that the returned Target has an ID and the centroids match
  EXPECT_GE(srv.response.target.id, 0);
  EXPECT_EQ(centroid.x, srv.response.target.centroid.x);
  EXPECT_EQ(centroid.y, srv.response.target.centroid.y);
  EXPECT_EQ(centroid.z, srv.response.target.centroid.z);
}

TEST_F(ServiceTest, targetIsAlreadyInJail) {
  clutter_butter::NewTarget srv1;
  geometry_msgs::Point centroid;
  centroid.x = 0.1;
  centroid.y = 0.1;
  centroid.z = 0.0;
  srv1.request.centroid = centroid;

  bool added = add_target.call(srv1);
  EXPECT_FALSE(added);

  // test that no plan is made
  clutter_butter::GetPushPlan srv2;
  bool has_plan = get_plan.call(srv2);
  EXPECT_EQ(0, srv2.response.plan.target.id);
  EXPECT_FALSE(has_plan);
}

TEST_F(ServiceTest, straitLineFromTargetToJail) {

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
  EXPECT_EQ(clutter_butter::GetPushPlanResponse::VALID, srv2.response.isvalid);

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
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
