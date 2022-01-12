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
/// \file
/// \brief This file includes extensions for the common transform functionality for autoware_auto_msgs
///        This file is created seperately from tf2_autoware_auto_msgs.hpp to preserve seperation of carma-platform changes from autoware.auto

#ifndef AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_EXTENSIONS_HPP_
#define AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_EXTENSIONS_HPP_

#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <kdl/frames.hpp>
#include <common/types.hpp>
#include <string>
#include <limits>


using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObjectKinematics = autoware_auto_msgs::msg::DetectedObjectKinematics;
using Shape = autoware_auto_msgs::msg::Shape;


namespace tf2
{

/********************/
/** DetectedObjects **/
/********************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs DetectedObjects type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The DetectedObjects message to transform.
 * \param t_out The transformed DetectedObjects message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const DetectedObjects & t_in, DetectedObjects & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;

  for (size_t i=0; i < t_in.objects.size(); ++i) {
    doTransform(t_in.objects[i], t_out.objects[i], transform);
  }
  
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
  doTransform(t_in.kinematics, t_out.kinematics, transform);
  doTransform(t_in.shape, t_out.shape, transform);
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

#endif  // AUTOWARE_AUTO_TF2__TF2_AUTOWARE_AUTO_MSGS_EXTENSIONS_HPP_
