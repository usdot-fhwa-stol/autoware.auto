/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */


#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs_extension.hpp>
#include <rclcpp/clock.hpp>
#include <memory>

// Forward declare filled_transform
geometry_msgs::msg::TransformStamped filled_transfom();

constexpr double eps = 1e-3;

TEST(Tf2AutowareAuto, DoTransformShape)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::Shape shape;
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;
  poly.points.push_back(p1);

  shape.polygon = poly;
  shape.height = 0;

  // doTransform
  autoware_auto_msgs::msg::Shape shape_out;
  tf2::doTransform(shape, shape_out, trans);

  ASSERT_EQ(shape_out.polygon.points.size(), 1u);
  EXPECT_NEAR(shape_out.polygon.points[0].x, 11, eps);
  EXPECT_NEAR(shape_out.polygon.points[0].y, 18, eps);
  EXPECT_NEAR(shape_out.polygon.points[0].z, 27, eps);
  EXPECT_NEAR(shape_out.height, 0, eps);
}

TEST(Tf2AutowareAuto, DoTransformDetectedObjectKinematics)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::DetectedObjectKinematics  dok;
  dok.orientation.w = 0;
  dok.orientation.x = 0;
  dok.orientation.y = 0;
  dok.orientation.z = 1;
  dok.centroid_position.x = 1;
  dok.centroid_position.y = 2;
  dok.centroid_position.z = 3;
  dok.twist.twist.linear.x = 1;
  dok.twist.twist.linear.y = 2;
  dok.twist.twist.linear.z = 3;
  dok.twist.twist.angular.x = 4;
  dok.twist.twist.angular.y = 5;
  dok.twist.twist.angular.z = 6;


  // doTransform
  autoware_auto_msgs::msg::DetectedObjectKinematics dok_out;
  tf2::doTransform(dok, dok_out, trans);

  EXPECT_NEAR(dok_out.orientation.w, 0.0, eps);
  EXPECT_NEAR(dok_out.orientation.x, 0.0, eps);
  EXPECT_NEAR(dok_out.orientation.y, -1.0, eps);
  EXPECT_NEAR(dok_out.orientation.z, 0.0, eps);
  EXPECT_NEAR(dok_out.centroid_position.x, 11, eps);
  EXPECT_NEAR(dok_out.centroid_position.y, 18, eps);
  EXPECT_NEAR(dok_out.centroid_position.z, 27, eps);

  // Verify twist is unchanged
  EXPECT_NEAR(dok_out.twist.twist.linear.x, 1, eps);
  EXPECT_NEAR(dok_out.twist.twist.linear.y, 2, eps);
  EXPECT_NEAR(dok_out.twist.twist.linear.z, 3, eps);
  EXPECT_NEAR(dok_out.twist.twist.angular.x, 4, eps);
  EXPECT_NEAR(dok_out.twist.twist.angular.y, 5, eps);
  EXPECT_NEAR(dok_out.twist.twist.angular.z, 6, eps);


  // Testing unused fields are unmodified
  EXPECT_EQ(dok_out.has_position_covariance, dok.has_position_covariance);
  EXPECT_EQ(dok_out.orientation_availability, dok.orientation_availability);
  EXPECT_EQ(dok_out.has_twist, dok.has_twist);
  EXPECT_EQ(dok_out.has_twist_covariance, dok.has_twist_covariance);

}

TEST(Tf2AutowareAuto, TransformDetectedObject)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::DetectedObject obj;
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;
  poly.points.push_back(p1);

  obj.shape.polygon = poly;
  obj.shape.height = 0;

  obj.kinematics.orientation.w = 0;
  obj.kinematics.orientation.x = 0;
  obj.kinematics.orientation.y = 0;
  obj.kinematics.orientation.z = 1;
  obj.kinematics.centroid_position.x = 1;
  obj.kinematics.centroid_position.y = 2;
  obj.kinematics.centroid_position.z = 3;
  obj.kinematics.twist.twist.linear.x = 1;
  obj.kinematics.twist.twist.linear.y = 2;
  obj.kinematics.twist.twist.linear.z = 3;
  obj.kinematics.twist.twist.angular.x = 4;
  obj.kinematics.twist.twist.angular.y = 5;
  obj.kinematics.twist.twist.angular.z = 6;

  // doTransform
  autoware_auto_msgs::msg::DetectedObject obj_out;
  tf2::doTransform(obj, obj_out, trans);

  ASSERT_EQ(obj_out.shape.polygon.points.size(), 1u);
  EXPECT_NEAR(obj_out.shape.polygon.points[0].x, 11, eps);
  EXPECT_NEAR(obj_out.shape.polygon.points[0].y, 18, eps);
  EXPECT_NEAR(obj_out.shape.polygon.points[0].z, 27, eps);
  EXPECT_NEAR(obj_out.shape.height, 0, eps);

  EXPECT_NEAR(obj_out.kinematics.orientation.w, 0.0, eps);
  EXPECT_NEAR(obj_out.kinematics.orientation.x, 0.0, eps);
  EXPECT_NEAR(obj_out.kinematics.orientation.y, -1.0, eps);
  EXPECT_NEAR(obj_out.kinematics.orientation.z, 0.0, eps);
  EXPECT_NEAR(obj_out.kinematics.centroid_position.x, 11, eps);
  EXPECT_NEAR(obj_out.kinematics.centroid_position.y, 18, eps);
  EXPECT_NEAR(obj_out.kinematics.centroid_position.z, 27, eps);

  // Verify twist is unchanged
  EXPECT_NEAR(obj_out.kinematics.twist.twist.linear.x, 1, eps);
  EXPECT_NEAR(obj_out.kinematics.twist.twist.linear.y, 2, eps);
  EXPECT_NEAR(obj_out.kinematics.twist.twist.linear.z, 3, eps);
  EXPECT_NEAR(obj_out.kinematics.twist.twist.angular.x, 4, eps);
  EXPECT_NEAR(obj_out.kinematics.twist.twist.angular.y, 5, eps);
  EXPECT_NEAR(obj_out.kinematics.twist.twist.angular.z, 6, eps);


  // Testing unused fields are unmodified
  EXPECT_EQ(obj_out.kinematics.has_position_covariance, obj.kinematics.has_position_covariance);
  EXPECT_EQ(obj_out.kinematics.orientation_availability, obj.kinematics.orientation_availability);
  EXPECT_EQ(obj_out.kinematics.has_twist, obj.kinematics.has_twist);
  EXPECT_EQ(obj_out.kinematics.has_twist_covariance, obj.kinematics.has_twist_covariance);

  // Object fields
  EXPECT_EQ(obj_out.existence_probability, obj.existence_probability);

}

TEST(Tf2AutowareAuto, TransformDetectedObjects)
{
  const auto trans = filled_transfom();
  autoware_auto_msgs::msg::DetectedObjects objs;
  autoware_auto_msgs::msg::DetectedObject obj;
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p1;
  p1.x = 1;
  p1.y = 2;
  p1.z = 3;
  poly.points.push_back(p1);

  obj.shape.polygon = poly;
  obj.shape.height = 0;

  obj.kinematics.orientation.w = 0;
  obj.kinematics.orientation.x = 0;
  obj.kinematics.orientation.y = 0;
  obj.kinematics.orientation.z = 1;
  obj.kinematics.centroid_position.x = 1;
  obj.kinematics.centroid_position.y = 2;
  obj.kinematics.centroid_position.z = 3;
  obj.kinematics.twist.twist.linear.x = 1;
  obj.kinematics.twist.twist.linear.y = 2;
  obj.kinematics.twist.twist.linear.z = 3;
  obj.kinematics.twist.twist.angular.x = 4;
  obj.kinematics.twist.twist.angular.y = 5;
  obj.kinematics.twist.twist.angular.z = 6;

  objs.objects.push_back(obj);
  objs.header.frame_id = "B";

  // doTransform
  autoware_auto_msgs::msg::DetectedObjects objs_out;
  tf2::doTransform(objs, objs_out, trans);

  // Partial field check here, relying on other tests for full verification
  ASSERT_TRUE(objs_out.header.frame_id == trans.header.frame_id);
  ASSERT_EQ(objs_out.objects.size(), 1u);
  ASSERT_EQ(objs_out.objects[0].shape.polygon.points.size(), 1u);
  EXPECT_NEAR(objs_out.objects[0].shape.polygon.points[0].x, 11, eps);
  EXPECT_NEAR(objs_out.objects[0].shape.polygon.points[0].y, 18, eps);
  EXPECT_NEAR(objs_out.objects[0].shape.polygon.points[0].z, 27, eps);
  EXPECT_NEAR(objs_out.objects[0].shape.height, 0, eps);
}
