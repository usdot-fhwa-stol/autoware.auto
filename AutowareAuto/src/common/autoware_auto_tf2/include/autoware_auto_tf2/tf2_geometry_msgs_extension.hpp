// Copyright 2008 Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/** \author Wim Meeussen */

/**
 * Modifications (C) Leidos 2022
 * - This file is a partial copy of tf2_geometry_msgs.hpp
 *   from https://github.com/ros2/geometry2/blob/7a660094d0da9c463be5fea1df60d836b3349e9b/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.hpp
 *
 *   In ROS2 Foxy only the stamped messages have their doTransform function implementations provided.
 *   In the linked updated file (post foxy) these implementations have been added.
 *   Therefore this file contains only the additions.
 *   The implementations provided in foxy are still assumed to come from the tf2_geometry_msgs.h file in that package.
 *
 *   NOTE: When carma-platform is updated to target a newer version of ROS2 this file should be removed
 *
 */

#ifndef TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_EXTENSION_HPP_
#define TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_EXTENSION_HPP_

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>

#include <array>
#include <string>

namespace tf2
{


/*************/
/** Vector3 **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a Vector3 message.
 * \param t_out The transformed vector, as a Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Vector3 & t_in,
  geometry_msgs::msg::Vector3 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v_out = gmTransformToKDL(transform).M * KDL::Vector(t_in.x, t_in.y, t_in.z);
  t_out.x = v_out[0];
  t_out.y = v_out[1];
  t_out.z = v_out[2];
}

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::msg::Vector3 toMsg(const tf2::Vector3 & in)
{
  geometry_msgs::msg::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::msg::Vector3 & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/***********/
/** Point **/
/***********/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point3 message.
 * \param t_out The transformed point, as a Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Point & t_in,
  geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v_out = gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
  t_out.x = v_out[0];
  t_out.y = v_out[1];
  t_out.z = v_out[2];
}

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::msg::Point & toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::msg::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/************************/
/** PoseWithCovariance **/
/************************/

// Forward declaration
void fromMsg(const geometry_msgs::msg::Transform & in, tf2::Transform & out);

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The pose to transform, as a Pose3 message with covariance.
 * \param t_out The transformed pose, as a Pose3 message with covariance.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::PoseWithCovariance & t_in,
  geometry_msgs::msg::PoseWithCovariance & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
  KDL::Rotation r = KDL::Rotation::Quaternion(
    t_in.pose.orientation.x, t_in.pose.orientation.y,
    t_in.pose.orientation.z, t_in.pose.orientation.w);

  KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
  t_out.pose.position.x = v_out.p[0];
  t_out.pose.position.y = v_out.p[1];
  t_out.pose.position.z = v_out.p[2];
  v_out.M.GetQuaternion(
    t_out.pose.orientation.x, t_out.pose.orientation.y,
    t_out.pose.orientation.z, t_out.pose.orientation.w);

  tf2::Transform tf_transform;
  fromMsg(transform.transform, tf_transform);
  t_out.covariance = transformCovariance(t_in.covariance, tf_transform);
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PoseWithCovariance message.
 * \return The input argument.
 */
inline
geometry_msgs::msg::PoseWithCovariance toMsg(const geometry_msgs::msg::PoseWithCovariance & in)
{
  return in;
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A PoseWithCovariance message.
 * \param out The input argument.
 */
inline
void fromMsg(
  const geometry_msgs::msg::PoseWithCovariance & msg,
  geometry_msgs::msg::PoseWithCovariance & out)
{
  out = msg;
}

/****************/
/** Quaternion **/
/****************/

// Forward declaration
geometry_msgs::msg::Quaternion toMsg(const tf2::Quaternion & in);

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a Quaternion3 message.
 * \param t_out The transformed quaternion, as a Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Quaternion & t_in,
  geometry_msgs::msg::Quaternion & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q_out = tf2::Quaternion(
    transform.transform.rotation.x, transform.transform.rotation.y,
    transform.transform.rotation.z, transform.transform.rotation.w) *
    tf2::Quaternion(t_in.x, t_in.y, t_in.z, t_in.w);
  t_out = toMsg(q_out);
}



/***************/
/** Transform **/
/***************/

/** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
 * \param in A tf2 Transform object.
 * \param out The Transform converted to a geometry_msgs message type.
 */
inline
/** This section is about converting */
void toMsg(const tf2::Transform & in, geometry_msgs::msg::Transform & out)
{
  out = toMsg(in);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Transform type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The frame to transform, as a Transform3 message.
 * \param t_out The frame transform, as a Transform3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Transform & t_in,
  geometry_msgs::msg::Transform & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v(t_in.translation.x, t_in.translation.y,
    t_in.translation.z);
  KDL::Rotation r = KDL::Rotation::Quaternion(
    t_in.rotation.x, t_in.rotation.y,
    t_in.rotation.z, t_in.rotation.w);

  KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
  t_out.translation.x = v_out.p[0];
  t_out.translation.y = v_out.p[1];
  t_out.translation.z = v_out.p[2];
  v_out.M.GetQuaternion(
    t_out.rotation.x, t_out.rotation.y,
    t_out.rotation.z, t_out.rotation.w);
}

/**********/
/** Pose **/
/**********/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The pose to transform, as a Pose3 message.
 * \param t_out The transformed pose, as a Pose3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Pose & t_in,
  geometry_msgs::msg::Pose & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v(t_in.position.x, t_in.position.y, t_in.position.z);
  KDL::Rotation r = KDL::Rotation::Quaternion(
    t_in.orientation.x, t_in.orientation.y,
    t_in.orientation.z, t_in.orientation.w);

  KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
  t_out.position.x = v_out.p[0];
  t_out.position.y = v_out.p[1];
  t_out.position.z = v_out.p[2];
  v_out.M.GetQuaternion(
    t_out.orientation.x, t_out.orientation.y,
    t_out.orientation.z, t_out.orientation.w);
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Pose message.
 * \return The input argument.
 */
inline
geometry_msgs::msg::Pose toMsg(const geometry_msgs::msg::Pose & in)
{
  return in;
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A Pose message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::msg::Pose & msg, geometry_msgs::msg::Pose & out)
{
  out = msg;
}

/**********************/
/*** WrenchStamped ****/
/**********************/

/** \brief Extract a timestamp from the header of a Wrench message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t WrenchStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template<>
inline
tf2::TimePoint getTimestamp(const geometry_msgs::msg::WrenchStamped & t)
{
  return tf2_ros::fromMsg(t.header.stamp);
}

/** \brief Extract a frame ID from the header of a Wrench message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t WrenchStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template<>
inline
std::string getFrameId(const geometry_msgs::msg::WrenchStamped & t) {return t.header.frame_id;}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Wrench type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The wrench to transform, as a Wrench message.
 * \param t_out The transformed wrench, as a Wrench message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Wrench & t_in, geometry_msgs::msg::Wrench & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  doTransform(t_in.force, t_out.force, transform);
  doTransform(t_in.torque, t_out.torque, transform);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs WrenchStamped type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The wrench to transform, as a timestamped Wrench message.
 * \param t_out The transformed wrench, as a timestamped Wrench message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::WrenchStamped & t_in,
  geometry_msgs::msg::WrenchStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  doTransform(t_in.wrench, t_out.wrench, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Trivial "conversion" function for Wrench message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A WrenchStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::msg::WrenchStamped toMsg(const geometry_msgs::msg::WrenchStamped & in)
{
  return in;
}

/** \brief Trivial "conversion" function for Wrench message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A WrenchStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::msg::WrenchStamped & msg, geometry_msgs::msg::WrenchStamped & out)
{
  out = msg;
}

}  // namespace tf2

#endif  // TF2_GEOMETRY_MSGS__TF2_GEOMETRY_MSGS_EXTENSION_HPP_
