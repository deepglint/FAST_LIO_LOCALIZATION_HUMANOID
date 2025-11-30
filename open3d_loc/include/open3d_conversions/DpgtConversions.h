#ifndef PROJECT_DPGTCONVERSIONS_HPP
#define PROJECT_DPGTCONVERSIONS_HPP

#include <string>
#include <iostream>
#include <memory>

// ROS 2 Headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp> // 用于 tf2 和 Eigen 的转换

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Eigen::Translation3d, Eigen::AngleAxisd

/**
 * @brief Transforms a Pose from one frame to another using tf2.
 * * @param tf_buffer Pointer to the tf2_ros::Buffer.
 * @param parent_frame_id Target frame ID.
 * @param child_frame_id Source frame ID.
 * @param input_pose The pose in child_frame_id.
 * @param output_pose The transformed pose in parent_frame_id.
 * @return true on successful transform, false otherwise.
 */
bool transform_pose(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                    const std::string &parent_frame_id,
                    const std::string &child_frame_id,
                    const geometry_msgs::msg::Pose &input_pose,
                    geometry_msgs::msg::Pose &output_pose)
{
  geometry_msgs::msg::PoseStamped tmp_input_pose, tmp_output_pose;

  // ROS 2: Time(0) 替换为 rclcpp::Time(0) 或使用 ROS 2 推荐的 tf2 查找最新变换
  // ROS 2: 使用 tf2::TimePointZero 代替 ros::Time(0) 查找最新变换
  tmp_input_pose.header.stamp = rclcpp::Time(0);
  tmp_output_pose.header.stamp = rclcpp::Time(0);
  tmp_input_pose.header.frame_id = child_frame_id;
  tmp_input_pose.pose = input_pose;

  // ROS 2: 日志宏现在包含在 rclcpp 中
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tf_conversion"), "child_frame_id = " << child_frame_id);

  try
  {
    // ROS 2: tf2::Buffer::canTransform() 替换 waitForTransform
    // ROS 2: tf2::Buffer::transform() 替换 tf::TransformListener::transformPose

    // 1. 等待变换 (可选，tf2::transform 会自动等待，但这里保留等待逻辑)
    if (!tf_buffer->canTransform(parent_frame_id, child_frame_id, tf2::TimePointZero, tf2::Duration(500000000))) // 0.5s in ns
    {
      RCLCPP_ERROR(rclcpp::get_logger("tf_conversion"), "Transform timeout between %s and %s.", parent_frame_id.c_str(), child_frame_id.c_str());
      return false;
    }

    // 2. 执行变换
    tmp_output_pose = tf_buffer->transform(tmp_input_pose, parent_frame_id, tf2::Duration(500000000));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("tf_conversion"), "Transform pose failure: " << ex.what());
    return false;
  }

  output_pose = tmp_output_pose.pose;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tf_conversion"), "output_pose: ["
                                                              << tmp_output_pose.pose.position.x << ", "
                                                              << tmp_output_pose.pose.position.y << ", "
                                                              << tmp_output_pose.pose.position.z << "]");
  return true;
}

/**
 * @brief Converts an Eigen::Matrix4d to a tf2::TransformStamped.
 * * @param Tm 4x4 Eigen Transformation Matrix.
 * @param transform Output tf2::TransformStamped.
 * @param parent_frame_id Parent frame ID.
 * @param child_frame_id Child frame ID.
 */
void MatrixToTransform(const Eigen::Matrix4d &Tm,
                       geometry_msgs::msg::TransformStamped &transform, // ROS 2 使用 TransformStamped 消息
                       const std::string &parent_frame_id = "parent_link",
                       const std::string &child_frame_id = "child_link")
{
  // ROS 2: 优先使用 tf2::toMsg/fromMsg 或 tf2_eigen 库进行转换
  Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();
  eigen_transform.matrix() = Tm;

  // 使用 tf2_eigen 库进行转换
  transform = tf2::eigenToTransform(eigen_transform);

  // 填充 Stamped 字段 (ROS 2 中通常需要一个 Node 来获取时间，这里简化为 rclcpp::Clock::now())
  // 假设调用者会在一个 rclcpp::Node 的上下文中
  transform.header.stamp = rclcpp::Clock().now();
  transform.header.frame_id = parent_frame_id;
  transform.child_frame_id = child_frame_id;
}

/**
 * @brief Converts an Eigen::Matrix4d to a tf2::Transform.
 * * @param Tm 4x4 Eigen Transformation Matrix.
 * @param transform Output tf2::Transform.
 */
void MatrixToTransform(const Eigen::Matrix4d &Tm,
                       tf2::Transform &transform) // ROS 2 使用 tf2::Transform
{
  // ROS 2: 使用 tf2::eigenToTF 宏或手动转换。这里使用 tf2::fromMsg
  Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();
  eigen_transform.matrix() = Tm;

  // 将 Eigen::Affine3d 转换为 geometry_msgs::msg::TransformStamped
  geometry_msgs::msg::TransformStamped tmp_msg_stamped = tf2::eigenToTransform(eigen_transform);

  // 再将 geometry_msgs::msg::Transform 转换为 tf2::Transform
  tf2::fromMsg(tmp_msg_stamped.transform, transform);
}

// 保持不变，因为是纯 Eigen/C++ 打印
void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

/**
 * @brief Converts a tf2::Transform to an Eigen::Matrix4d.
 * * @param transform Input tf2::Transform.
 * @param transform_matrix Output 4x4 Eigen Transformation Matrix.
 */
void TransformToMatrix(const tf2::Transform &transform, // ROS 2 使用 tf2::Transform
                       Eigen::Matrix4d &transform_matrix)
{
  // ROS 2: 手动构建 Eigen::Affine3d 从 tf2::Transform
  Eigen::Affine3d eigen_transform;

  // 提取平移部分
  eigen_transform.translation() = Eigen::Vector3d(
      transform.getOrigin().x(),
      transform.getOrigin().y(),
      transform.getOrigin().z());

  // 提取旋转部分
  Eigen::Quaterniond q(
      transform.getRotation().w(),
      transform.getRotation().x(),
      transform.getRotation().y(),
      transform.getRotation().z());
  eigen_transform.linear() = q.toRotationMatrix();

  transform_matrix = eigen_transform.matrix();

  // **注意:** 原来的 tf::Matrix3x3::getEulerYPR 逻辑是 ZYX 欧拉角 (Yaw-Pitch-Roll)，
  // 而 Eigen::AngleAxisd 的乘法顺序 (tl_btol * rot_z * rot_y * rot_x) 也是 ZYX，
  // 因此在 ROS 2 中，手动构建 Eigen::Affine3d 是最直接的方式。
}

/**
 * @brief Converts Euler angles (Roll, Pitch, Yaw) to a tf2::Quaternion.
 * * @param roll Roll angle (rad).
 * @param pitch Pitch angle (rad).
 * @param yaw Yaw angle (rad).
 * @return tf2::Quaternion
 */
tf2::Quaternion euler2Quaternion(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q; // ROS 2 使用 tf2::Quaternion
  q.setRPY(roll, pitch, yaw);
  std::cout << "Euler2Quaternion result is:" << std::endl;
  std::cout << "x = " << q.getX() << std::endl; // tf2 使用 getX()
  std::cout << "y = " << q.getY() << std::endl;
  std::cout << "z = " << q.getZ() << std::endl;
  std::cout << "w = " << q.getW() << std::endl
            << std::endl;
  return q;
}

/**
 * @brief Converts Quaternion (x, y, z, w) to Eigen::Vector3d Euler angles (Roll, Pitch, Yaw).
 * * @param x Quaternion X component.
 * @param y Quaternion Y component.
 * @param z Quaternion Z component.
 * @param w Quaternion W component.
 * @return Eigen::Vector3d containing [Roll, Pitch, Yaw] in radians.
 */
Eigen::Vector3d Quaterniond2Euler(const double x, const double y, const double z, const double w)
{
  tf2::Quaternion quat;
  geometry_msgs::msg::Pose pose; // ROS 2 消息类型
  pose.orientation.x = x;
  pose.orientation.y = y;
  pose.orientation.z = z;
  pose.orientation.w = w;

  // ROS 2: 使用 tf2::fromMsg 将消息转换为 tf2 类型
  tf2::fromMsg(pose.orientation, quat);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // tf2::Matrix3x3::getRPY

  Eigen::Vector3d euler;
  euler[0] = roll;  // Roll
  euler[1] = pitch; // Pitch
  euler[2] = yaw;   // Yaw

  return euler;
}

#endif // PROJECT_DPGTCONVERSIONS_HPP