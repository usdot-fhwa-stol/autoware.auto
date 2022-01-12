// Copyright 2020, The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/// \file
/// \brief This file includes common transform functionality for autoware_auto_msgs

/**
 * Copyright (C) 2021 LEIDOS.
 * 
 * Modifications
 * - Add DetectedObjects to the set of supporte types
 */ 

#ifndef AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_
#define AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_

#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <autoware_auto_msgs/msg/quaternion32.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <kdl/frames.hpp>
#include <common/types.hpp>
#include <string>
#include <limits>


using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
using BoundingBox = autoware_auto_msgs::msg::BoundingBox;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObjectKinematics = autoware_auto_msgs::msg::DetectedObjectKinematics;
using Shape = autoware_auto_msgs::msg::Shape;


namespace tf2
{


/*************/
/** Point32 **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point32 message.
 * \param t_out The transformed point, as a Point32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Point32 & t_in, geometry_msgs::msg::Point32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const KDL::Vector v_out = gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
  t_out.x = static_cast<float>(v_out[0]);
  t_out.y = static_cast<float>(v_out[1]);
  t_out.z = static_cast<float>(v_out[2]);
}


/*************/
/** Polygon **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Polygon type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The polygon to transform.
 * \param t_out The transformed polygon.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Polygon & t_in, geometry_msgs::msg::Polygon & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  // Don't call the Point32 doTransform to avoid doing this conversion every time
  const auto kdl_frame = gmTransformToKDL(transform);
  // We don't use std::back_inserter to allow aliasing between t_in and t_out
  t_out.points.resize(t_in.points.size());
  for (size_t i = 0; i < t_in.points.size(); ++i) {
    const KDL::Vector v_out = kdl_frame * KDL::Vector(
      t_in.points[i].x, t_in.points[i].y, t_in.points[i].z);
    t_out.points[i].x = static_cast<float>(v_out[0]);
    t_out.points[i].y = static_cast<float>(v_out[1]);
    t_out.points[i].z = static_cast<float>(v_out[2]);
  }
}

/******************/
/** Quaternion32 **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs Quaternion32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The Quaternion32 message to transform.
 * \param t_out The transformed Quaternion32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const autoware_auto_msgs::msg::Quaternion32 & t_in,
  autoware_auto_msgs::msg::Quaternion32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Rotation r_in = KDL::Rotation::Quaternion(t_in.x, t_in.y, t_in.z, t_in.w);
  KDL::Rotation out = gmTransformToKDL(transform).M * r_in;

  double qx, qy, qz, qw;
  out.GetQuaternion(qx, qy, qz, qw);
  t_out.x = static_cast<float32_t>(qx);
  t_out.y = static_cast<float32_t>(qy);
  t_out.z = static_cast<float32_t>(qz);
  t_out.w = static_cast<float32_t>(qw);
}


/******************/
/** BoundingBox **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBox type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBox message to transform.
 * \param t_out The transformed BoundingBox message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const BoundingBox & t_in, BoundingBox & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  doTransform(t_in.orientation, t_out.orientation, transform);
  doTransform(t_in.centroid, t_out.centroid, transform);
  doTransform(t_in.corners[0], t_out.corners[0], transform);
  doTransform(t_in.corners[1], t_out.corners[1], transform);
  doTransform(t_in.corners[2], t_out.corners[2], transform);
  doTransform(t_in.corners[3], t_out.corners[3], transform);
  // TODO(jitrc): add conversion for other fields of BoundingBox, such as heading, variance, size
}


/**********************/
/** BoundingBoxArray **/
/**********************/

/** \brief Extract a timestamp from the header of a BoundingBoxArray message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template<>
inline
tf2::TimePoint getTimestamp(const BoundingBoxArray & t)
{
  return tf2_ros::fromMsg(t.header.stamp);
}

/** \brief Extract a frame ID from the header of a BoundingBoxArray message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template<>
inline
std::string getFrameId(const BoundingBoxArray & t) {return t.header.frame_id;}

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBoxArray type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBoxArray to transform, as a timestamped BoundingBoxArray message.
 * \param t_out The transformed BoundingBoxArray, as a timestamped BoundingBoxArray message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const BoundingBoxArray & t_in,
  BoundingBoxArray & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  for (auto idx = 0U; idx < t_in.boxes.size(); idx++) {
    doTransform(t_out.boxes[idx], t_out.boxes[idx], transform);
  }
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/********************/
/** DetectedObject **/
/********************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs DetectedObject type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The DetectedObject message to transform.
 * \param t_out The transformed DetectedObject message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const DetectedObject & t_in, DetectedObject & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  doTransform(t_in.orientation, t_out.orientation, transform);
  doTransform(t_in.centroid, t_out.centroid, transform);
  doTransform(t_in.corners[0], t_out.corners[0], transform);
  doTransform(t_in.corners[1], t_out.corners[1], transform);
  doTransform(t_in.corners[2], t_out.corners[2], transform);
  doTransform(t_in.corners[3], t_out.corners[3], transform);
  // TODO(jitrc): add conversion for other fields of BoundingBox, such as heading, variance, size
}

/******************************/
/** DetectedObjectKinematics **/
/******************************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs DetectedObjectKinematics type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The DetectedObjectKinematics message to transform.
 * \param t_out The transformed DetectedObjectKinematics message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const DetectedObjectKinematics & t_in, DetectedObjectKinematics & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in; // Copy un-transformable fields
  // Transform geometric fields
  doTransform(t_in.centroid_position, t_out.centroid_position, transform);
  doTransform(t_in.orientation, t_out.orientation, transform);
  doTransform(t_in.twist, t_out.twist, transform);
  
  // Transform position covariance if available
  if (t_in.has_position_covariance) {
    // To transform the covariance we will build a new PoseWithCovariance message
    // since the tf2_geometry_msgs package contains a transformation for that covariance
    geometry_msgs::msg::PoseWithCovariance cov_pose_in;
    geometry_msgs::msg::PoseWithCovariance cov_pose_out;

    // The DetectedObjectKinematics covariance is 9 element position. 
    // This needs to be mapped onto the 36 element PoseWithCovariance covariance.
    auto xx = t_in.kinematics.position_covariance[0];
    auto xy = t_in.kinematics.position_covariance[1];
    auto xz = t_in.kinematics.position_covariance[2];
    auto yx = t_in.kinematics.position_covariance[3];
    auto yy = t_in.kinematics.position_covariance[4];
    auto yz = t_in.kinematics.position_covariance[5];
    auto zx = t_in.kinematics.position_covariance[6];
    auto zy = t_in.kinematics.position_covariance[7];
    auto zz = t_in.kinematics.position_covariance[8];

    // This matrix represents the covariance of the object before transformation
    std::array<double, 36> input_covariance = { 
      xx, xy, xz,  0, 0, 0,
      yx, yy, yz,  0, 0, 0,
      zx, zy, zz,  0, 0, 0,
      0,  0,  0,  1,  0, 0, // Since no covariance for the orientation is provided we will assume an identity relationship (1s on the diagonal)
      0,  0,  0,  0,  1, 0, 
      0,  0,  0,  0,  0, 1
    };

    cov_pose_in.covariances = input_covariance;

    doTransform(cov_pose_in, cov_pose_out, transform);
    
    // Copy the transformed covariance into the output message

    t_out.kinematics.position_covariance[0] = cov_pose_in.covariances[0];
    t_out.kinematics.position_covariance[1] = cov_pose_in.covariances[1];
    t_out.kinematics.position_covariance[2] = cov_pose_in.covariances[2];
    t_out.kinematics.position_covariance[3] = cov_pose_in.covariances[6];
    t_out.kinematics.position_covariance[4] = cov_pose_in.covariances[7];
    t_out.kinematics.position_covariance[5] = cov_pose_in.covariances[8];
    t_out.kinematics.position_covariance[6] = cov_pose_in.covariances[12];
    t_out.kinematics.position_covariance[7] = cov_pose_in.covariances[13];
    t_out.kinematics.position_covariance[8] = cov_pose_in.covariances[14];
  }
}

/***********/
/** Shape **/
/***********/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs Shape type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The Shape message to transform.
 * \param t_out The transformed Shape message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const Shapes & t_in, Shape & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in; // Copy un-transformable fields

  // Transform polygon
  doTransform(t_in.polygon, t_out.polygon, transform);

  // Correct height field based on transformed polygon. 
  // Z is always gravity-aligned according to the message spec
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
  
  for (auto p : t_out.polygon.points) {
    if (p.z < min_z) {
      min_z = p.z;
    }
    if (p.z > max_z) {
      max_z = p.z;
    }
  }

  t_out.height = std::abs(max_z - min_z);
}


}  // namespace tf2

#endif  // AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_HPP_
