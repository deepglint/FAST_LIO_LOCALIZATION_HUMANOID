#ifndef PROJECT_ROSCONVERSION_H
#define PROJECT_ROSCONVERSION_H

#include <algorithm>
#include <vector>
#include <string>
#include <istream>
#include <cmath>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Transform.h> // 这个在ROS2中仍然存在
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <urdf_model/model.h>

#ifdef CRCL
// CRCL representations
#include "RCS.h"
#include <sensor_msgs/msg/joint_state.hpp>
using JointState = sensor_msgs::msg::JointState;
#endif

// Error at compile time for non handled convert
#include <type_traits> // For static_assert

#ifndef Deg2Rad
#define Deg2Rad(Ang) ((double)(Ang * M_PI / 180.0))
#define Rad2Deg(Ang) ((double)(Ang * 180.0 / M_PI))
#define MM2Meter(d) ((double)(d / 1000.00))
#define Meter2MM(d) ((double)(d * 1000.00))
#endif

namespace Conversion
{
    using tfPose = tf2::Transform;
    using tfQuaternion = tf2::Quaternion;
    using tfVector3 = tf2::Vector3;
    using tfMatrix3x3 = tf2::Matrix3x3;

    using PointMsg = geometry_msgs::msg::Point;
    using PoseMsg = geometry_msgs::msg::Pose;
    using QuaternionMsg = geometry_msgs::msg::Quaternion;
    using TransformMsg = geometry_msgs::msg::Transform;
    using TwistMsg = geometry_msgs::msg::Twist;
    using Vector3Msg = geometry_msgs::msg::Vector3;
    using WrenchMsg = geometry_msgs::msg::Wrench;

    /*!
     * \brief ToVector calling parameters MUST match e.g., double or long depending on template, OR wont work
     * \param n number of items to push into std vector.
     * \param ... list of n template type items
     * \return std vector of type T
     *
     * Note: Fixed to work only for double due to va_arg limitations.
     */
    inline std::vector<double> ToVector(int n, ...)
    {
        std::vector<double> ds;
        va_list args;      // define argument list variable
        va_start(args, n); // init list; point to last defined argument

        for (int i = 0; i < n; i++)
        {
            double d = va_arg(args, double); // get next argument
            ds.push_back(d);
        }
        va_end(args); // clean up the system stack
        return ds;
    }

    /*!
     * \brief Convert a list of std vector  type T from degrees to radians
     * Change template from typep into double conversion factor.
     *  you#ifdef CRCL can't use float literals as template parameters
     */
    template <typename T>
    inline std::vector<T> ScaleVector(std::vector<T> goaljts, double multiplier = M_PI / 180.0)
    {
        // transform angles from degree to radians
        std::transform(goaljts.begin(), goaljts.end(), goaljts.begin(),
                       [multiplier](T val)
                       { return static_cast<T>(static_cast<double>(val) * multiplier); });
        return goaljts;
    }

    /*!
     * \brief Empty conversion of type from into type to. If called, asserts.
     * \param f is defined in the template corresponding to the "From" typename.
     * \return to is defined in the template corresponding "To"  typename
     */
    template <typename From, typename To>
    inline To Convert(From f)
    {
        static_assert(sizeof(To) == 0, "Conversion not implemented");
        return To{};
    }

    /*!
     * \brief Convert a list of std vector  type string into a vector of doubles
     * \return converted set of std::vector<double> returned if error assume 0 put into array.
     */
    template <>
    inline std::vector<double> Convert<std::vector<std::string>, std::vector<double>>(
        std::vector<std::string> stringVector)
    {
        std::vector<double> doubleVector;
        std::transform(stringVector.begin(), stringVector.end(), std::back_inserter(doubleVector),
                       [](std::string const &val)
                       {
                           try
                           {
                               return std::stod(val);
                           }
                           catch (...)
                           {
                               return 0.0;
                           }
                       });
        return doubleVector;
    }

    template <typename To>
    inline std::vector<To> ConvertStringVector(std::vector<std::string> From)
    {
        std::vector<To> toVector;
        for (size_t i = 0; i < From.size(); i++)
        {
            std::istringstream ss(From[i]);
            To result = To{};
            if (!(ss >> result))
            {
                result = To{0};
            }
            toVector.push_back(result);
        }
        return toVector;
    }
    // tf2

    /*!
     * \brief Convert Eigen::Affine3d into tf2::Transform.
     * \param pose is copy constructor of Eigen::Affine3d.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<Eigen::Affine3d, tfPose>(Eigen::Affine3d pose)
    {
#if 1
        Eigen::Quaterniond q(pose.rotation());
        return tfPose(tfQuaternion(q.x(), q.y(), q.z(), q.w()),
                      tfVector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
#else
        tfPose p;
        p.setOrigin(tfVector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
        Eigen::Quaterniond q(pose.rotation());
        p.setRotation(tfQuaternion(q.x(), q.y(), q.z(), q.w()));
        return p;
#endif
    }

    /*!
     * \brief Convert geometry_msgs::msg::Pose into tf2::Transform.
     * \param pose is copy constructor of geometry_msgs::msg::Pose.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<PoseMsg, tfPose>(PoseMsg m)
    {
        return tfPose(tfQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                      tfVector3(m.position.x, m.position.y, m.position.z));
    }

    /*!
     * \brief Convert Eigen::Quaterniond into tf2::Quaternion.
     * \param e is copy constructor of Eigen::Quaterniond.
     * \return tf2::Quaternion
     */
    template <>
    inline tfQuaternion Convert<Eigen::Quaterniond, tfQuaternion>(Eigen::Quaterniond e)
    {
        tfQuaternion q;
        q.setX(e.x());
        q.setY(e.y());
        q.setZ(e.z());
        q.setW(e.w());
        return q;
    }

    /*!
     * \brief Convert std::vector<double> into tf2::Transform.
     * \param ds are an array of 7 doubles to define tf Transform.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<std::vector<double>, tfPose>(std::vector<double> ds)
    {
        assert(ds.size() > 6);
        return tfPose(tfQuaternion(ds[3], ds[4], ds[5], ds[6]), tfVector3(ds[0], ds[1], ds[2]));
    }

    /*!
     * \brief Convert std::vector<double> into tf2::Vector3.
     * \param ds are an array of 3 doubles to define tf Vector3.
     * \return tf2::Vector3
     */
    template <>
    inline tfVector3 Convert<std::vector<double>, tfVector3>(std::vector<double> ds)
    {
        assert(ds.size() > 2);
        return tfVector3(ds[0], ds[1], ds[2]);
    }

    /*!
     * \brief Convert tf2::Quaternion into tf2::Transform.
     * \param q rotation is converted into a tf2::Transform.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<tfQuaternion, tfPose>(tfQuaternion q)
    {
        return tfPose(q, tfVector3(0., 0., 0.));
    }

    /*!
     * \brief Convert tf2::Vector3 into tf2::Transform.
     * \param  t is translation is converted into a tf2::Transform.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<tfVector3, tfPose>(tfVector3 t)
    {
        return tfPose(tfQuaternion(0.0, 0.0, 0.0, 1.0), t);
    }

    /*!
     * \brief Convert Eigen::Matrix4d into tf2::Transform.
     * \param  m is Eigen 4x4 matrix to be converted into a tf2::Transform.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<Eigen::Matrix4d, tfPose>(Eigen::Matrix4d m)
    {
        tfPose pose;
        Eigen::Vector3d trans(m.block<3, 1>(0, 3));
        Eigen::Quaterniond q(m.block<3, 3>(0, 0));
        pose.setRotation(tfQuaternion(q.x(), q.y(), q.z(), q.w()));
        pose.setOrigin(tfVector3(trans.x(), trans.y(), trans.z()));
        return pose;
    }

    /*!
     * \brief Convert Eigen::Matrix4d into geometry_msgs::msg::Pose.
     * \param  m is Eigen 4x4 matrix to be converted into a geometry_msgs::msg::Pose.
     * \return geometry_msgs::msg::Pose
     */
    template <>
    inline PoseMsg Convert<Eigen::Matrix4d, PoseMsg>(Eigen::Matrix4d m)
    {
        tfPose pose;
        Eigen::Vector3d trans(m.block<3, 1>(0, 3));
        Eigen::Quaterniond q(m.block<3, 3>(0, 0));
        pose.setRotation(tfQuaternion(q.x(), q.y(), q.z(), q.w()));
        pose.setOrigin(tfVector3(trans.x(), trans.y(), trans.z()));

        PoseMsg result;
        result.position.x = pose.getOrigin().x();
        result.position.y = pose.getOrigin().y();
        result.position.z = pose.getOrigin().z();
        result.orientation.x = pose.getRotation().x();
        result.orientation.y = pose.getRotation().y();
        result.orientation.z = pose.getRotation().z();
        result.orientation.w = pose.getRotation().w();

        return result;
    }

    /*!
     * \brief Convert tf2::Transform into Eigen::Matrix4d.
     * \param  m is tf2::Transform to be converted into a Eigen::Matrix4d.
     * \return Eigen::Matrix4d
     */
    template <>
    inline Eigen::Matrix4d Convert<tfPose, Eigen::Matrix4d>(tfPose m)
    {
        Eigen::Matrix4d result;
        Eigen::Translation3d tl_btol(
            m.getOrigin().getX(),
            m.getOrigin().getY(),
            m.getOrigin().getZ());
        double roll, pitch, yaw;
        tfMatrix3x3(m.getRotation()).getRPY(roll, pitch, yaw);
        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
        result = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        return result;
    }

    template <>
    inline Eigen::Matrix4d Convert<PoseMsg, Eigen::Matrix4d>(PoseMsg m)
    {
        tfPose pose = tfPose(tfQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                             tfVector3(m.position.x, m.position.y, m.position.z));

        Eigen::Matrix4d result;
        Eigen::Translation3d tl_btol(
            pose.getOrigin().getX(),
            pose.getOrigin().getY(),
            pose.getOrigin().getZ());
        double roll, pitch, yaw;
        tfMatrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
        result = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
        return result;
    }

    template <>
    inline Eigen::Isometry3d Convert<PoseMsg, Eigen::Isometry3d>(PoseMsg m)
    {
        tfPose pose = tfPose(tfQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                             tfVector3(m.position.x, m.position.y, m.position.z));
        Eigen::Matrix4d mid;
        Eigen::Translation3d tl_btol(
            pose.getOrigin().getX(),
            pose.getOrigin().getY(),
            pose.getOrigin().getZ());
        double roll, pitch, yaw;
        tfMatrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
        mid = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        Eigen::Isometry3d result(mid);
        return result;
    }

    /*!
     * \brief CreateRPYPose taks array of double and create a tf2::Transform.
     * \param ds is a  std array of 6 doubles to create pose (rpy + xyz).
     * \return tf2::Transform
     */
    inline tfPose CreateRPYPose(std::vector<double> ds)
    {
        assert(ds.size() > 5);
        tfQuaternion q;
        q.setRPY(ds[3], ds[4], ds[5]); // roll, pitch, yaw
        return tfPose(q, tfVector3(ds[0], ds[1], ds[2]));
    }

    /*!
     * \brief Create Pose from a axis and angle rotation representation.
     * \param axis is the unit vector to rotation around.
     * \param angle is the angle of rotation in radians.
     * \return tf2::Transform
     */
    inline tfPose CreatePose(tfVector3 axis, double angle)
    {
        return tfPose(tfQuaternion(axis, angle), tfVector3(0.0, 0.0, 0.0));
    }

    /*!
     * \brief Create Quaternion from a rpy rotation representation designated in radians.
     * \param roll rotation around x axis in radians.
     * \param pitch rotation around y axis in radians.
     * \param yaw rotation around z axis in radians.
     * \return tf2::Quaternion
     */
    inline tfQuaternion RPYRadians(double roll, double pitch, double yaw)
    {
        tfQuaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }

    /*!
     * \brief Create Quaternion from a rpy rotation representation designated in degrees.
     * \param roll rotation around x axis in degrees.
     * \param pitch rotation around y axis in degrees.
     * \param yaw rotation around z axis in degrees.
     * \return tf2::Quaternion
     */
    inline tfQuaternion RPYDegrees(double roll, double pitch, double yaw)
    {
        return RPYRadians(Deg2Rad(roll), Deg2Rad(pitch), Deg2Rad(yaw));
    }

    /*!
     * \brief Convert geometry_msgs::msg::Pose into tf2::Vector3.
     * \param e is copy constructor of geometry_msgs::msg::Pose.
     * \return tf2::Vector3
     */
    template <>
    inline tfVector3 Convert<PoseMsg, tfVector3>(PoseMsg e)
    {
        return tfVector3(e.position.x, e.position.y, e.position.z);
    }

    /*!
     * \brief Convert Eigen::Vector3d into tf2::Transform.
     * \param e is copy constructor of Eigen::Vector3d.
     * \return tf2::Transform
     */
    template <>
    inline tfPose Convert<Eigen::Vector3d, tfPose>(Eigen::Vector3d e)
    {
        return tfPose(tfQuaternion(0.0, 0.0, 0.0, 1.0), tfVector3(e(0), e(1), e(2)));
    }

    /*!
     * \brief Convert Eigen matrix into tf2::Vector3.
     * Example: tf2::Vector3 v = matrixEigenToTfVector<Eigen::Matrix3d>(m);
     * \param e is copy constructor of Eigen Matrix, either 3x3, 4x4, double or float.
     * \return tf2::Vector3
     */
    template <typename T>
    inline tfVector3 matrixEigenToTfVector(T e)
    {
        return tfVector3(e(0, 3), e(1, 3), e(2, 3));
    }

    /*!
     * \brief Convert Eigen::Vector3d into tf2::Vector3.
     * \param e is copy constructor of Eigen::Vector3d.
     * \return tf2::Vector3
     */
    template <>
    inline tfVector3 Convert<Eigen::Vector3d, tfVector3>(Eigen::Vector3d e)
    {
        return tfVector3(e(0), e(1), e(2));
    }

    /*!
     * \brief Convert Eigen::Matrix3d into tf2::Matrix3x3.
     * \param e is copy constructor of Eigen Matrix3d, a 3x3 double matrix.
     * \return tf2::Matrix3x3
     */
    template <>
    inline tfMatrix3x3 Convert<Eigen::Matrix3d, tfMatrix3x3>(Eigen::Matrix3d e)
    {
        tfMatrix3x3 t(e(0, 0), e(0, 1), e(0, 2),
                      e(1, 0), e(1, 1), e(1, 2),
                      e(2, 0), e(2, 1), e(2, 2));
        return t;
    }

    /*!
     * \brief Create Identity Transform
     * \return tf2::Transform
     */
    inline tfPose Identity()
    {
        return tfPose::getIdentity();
    }
    // Eigen

    /*!
     * \brief Convert<tf2::Transform, Eigen::Affine3d> converts tf transform into an  Eigen affine 4x4 matrix  o represent the pose
     * \param pose is the tf transform with position and orientation.
     * \return   Eigen Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<tfPose, Eigen::Affine3d>(tfPose pose)
    {
        Eigen::Quaterniond q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
        return Eigen::Affine3d(Eigen::Translation3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()) * q.toRotationMatrix());
    }

    /*!
     * \brief Convert Eigen::Quaternion into an  Eigen affine 4x4 matrix  o represent the pose
     * \param q is the Eigen::Quaternion orientation.
     * \return   Eigen Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<Eigen::Quaternion<double>, Eigen::Affine3d>(Eigen::Quaternion<double> q)
    {
        return Eigen::Affine3d::Identity() * q;
    }

    /*!
     * \brief Convert geometry_msgs::msg::Pose into an  Eigen affine3d 4x4 matrix  o represent the pose.
     * Uses tf conversion utilities.
     * \param m is defined as a geometry_msgs::msg::Pose..
     * \return  Eigen Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<PoseMsg, Eigen::Affine3d>(PoseMsg m)
    {
        Eigen::Affine3d e = Eigen::Translation3d(m.position.x,
                                                 m.position.y,
                                                 m.position.z) *
                            Eigen::Quaterniond(m.orientation.w,
                                               m.orientation.x,
                                               m.orientation.y,
                                               m.orientation.z);
        return e;
    }

    /*!
     * \brief Convert tf2::Quaternion into an  Eigen::Quaterniond.
     * \param q is defined as a tf2::Quaternion..
     * \return  Eigen::Quaterniond vector
     */
    template <>
    inline Eigen::Quaterniond Convert<tfQuaternion, Eigen::Quaterniond>(tfQuaternion q)
    {
        return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    /*!
     * \brief Convert geometry_msgs::msg::Pose into an  Eigen::Translation3d.
     * \param pose is defined as a geometry_msgs::msg::Pose..
     * \return  Eigen::Translation3d vector
     */
    template <>
    inline Eigen::Translation3d Convert<PoseMsg, Eigen::Translation3d>(PoseMsg pose)
    {
        return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z);
    }

    /*!
     * \brief Convert tf2::Vector3 into an  Eigen::Vector3d.
     * \param t is translation is defined as a tf2::Vector3..
     * \return  Eigen::Vector3d vector
     */
    template <>
    inline Eigen::Vector3d Convert<tfVector3, Eigen::Vector3d>(tfVector3 t)
    {
        return Eigen::Vector3d(t.getX(), t.getY(), t.getZ());
    }

    /*!
     * \brief Convert Eigen::Vector3d translation into an  Eigen::Affine3d pose.
     * \param translation is defined as a Eigen::Vector3d.
     * \return  Eigen::Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<Eigen::Vector3d, Eigen::Affine3d>(Eigen::Vector3d translation)
    {
        Eigen::Affine3d shared_pose_eigen_ = Eigen::Affine3d::Identity();
        shared_pose_eigen_.translation() = translation;
        return shared_pose_eigen_;
    }

    /*!
     * \brief Create Eigen::Affine3d as an axis angle definition around z axis.
     * \param zangle is angle of rotation in radians around Z.
     * \return  Eigen::Affine3d pose
     */
    inline Eigen::Affine3d CreateEigenPose(double zangle)
    {
        return Eigen::Affine3d::Identity() * Eigen::AngleAxisd(zangle, Eigen::Vector3d::UnitZ());
    }

    /*!
     * \brief Convert Eigen::Translation3d translation into an  Eigen::Affine3d pose.
     * \param t is translation  defined as a Eigen::Translation3d.
     * \return  Eigen::Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<Eigen::Translation3d, Eigen::Affine3d>(Eigen::Translation3d trans)
    {
        return Eigen::Affine3d::Identity() * trans;
    }

    /*!
     * \brief Convert geometry_msgs::msg::Point translation into an  Eigen::Vector3d vector.
     * \param point is translation  defined as a geometry_msgs::msg::Point.
     * \return  Eigen::Vector3d vector
     */
    template <>
    inline Eigen::Vector3d Convert<PointMsg, Eigen::Vector3d>(PointMsg point)
    {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    /*!
     * \brief Convert geometry_msgs::msg::Point translation into an  Eigen::Affine3d pose.
     * \param point is translation  defined as a geometry_msgs::msg::Point.
     * \return  Eigen::Affine3d pose
     */
    template <>
    inline Eigen::Affine3d Convert<PointMsg, Eigen::Affine3d>(PointMsg point)
    {
        return Eigen::Affine3d::Identity() * Eigen::Translation3d(point.x, point.y, point.z);
    }

    /*!
     * \brief Convert Eigen::Affine3d  into an  Eigen::Vector3d vector.
     * \param e is  pose defined as a Eigen Affine3d.
     * \return  Eigen::Vector3d vector
     */
    template <>
    inline Eigen::Vector3d Convert<Eigen::Affine3d, Eigen::Vector3d>(Eigen::Affine3d e)
    {
        return Eigen::Vector3d(e.matrix()(0, 3), e.matrix()(1, 3), e.matrix()(2, 3));
    }

    // geometry_msgs - constructor nightmare.

    /*!
     * \brief Convert tf2::Transform pose into an  geometry_msgs::msg::Pose pose.
     * \param m is a tf2::Transform transform matrix.
     * \return  geometry_msgs::msg::Pose pose
     */
    template <>
    inline PoseMsg Convert<tfPose, PoseMsg>(tfPose m)
    {
        PoseMsg p;
        p.position.x = m.getOrigin().getX();
        p.position.y = m.getOrigin().getY();
        p.position.z = m.getOrigin().getZ();
        p.orientation.x = m.getRotation().getX();
        p.orientation.y = m.getRotation().getY();
        p.orientation.z = m.getRotation().getZ();
        p.orientation.w = m.getRotation().getW();
        return p;
    }

    /*!
     * \brief Convert geometry_msgs::msg::Point point into an  geometry_msgs::msg::Pose pose.
     * \param point geometry_msgs::msg::Point is translation.
     * \return  geometry_msgs::msg::Pose pose.
     */
    template <>
    inline PoseMsg Convert<PointMsg, PoseMsg>(PointMsg point)
    {
        PoseMsg shared_pose_msg_;
        shared_pose_msg_.orientation.x = 0.0;
        shared_pose_msg_.orientation.y = 0.0;
        shared_pose_msg_.orientation.z = 0.0;
        shared_pose_msg_.orientation.w = 1.0;
        shared_pose_msg_.position = point;
        return shared_pose_msg_;
    }

    /*!
     * \brief Convert Eigen::Vector3d point into an geometry_msgs::msg::Point position vector.
     * \param point Eigen::Vector3d is translation.
     * \return  geometry_msgs::msg::Point position vector.
     */
    template <>
    inline PointMsg Convert<Eigen::Vector3d, PointMsg>(Eigen::Vector3d point)
    {
        PointMsg pt;
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        return pt;
    }

    /*!
     * \brief Convert Eigen::Affine3d pose into an geometry_msgs::msg::Pose pose.
     * \param e is Eigen::Affine3d defining equivalent pose.
     * \return  geometry_msgs::msg::Pose pose.
     */
    template <>
    inline PoseMsg Convert<Eigen::Affine3d, PoseMsg>(Eigen::Affine3d e)
    {
        PoseMsg m;
        m.position.x = e.translation().x();
        m.position.y = e.translation().y();
        m.position.z = e.translation().z();
        Eigen::Quaterniond q = Eigen::Quaterniond(e.linear());
        m.orientation.x = q.x();
        m.orientation.y = q.y();
        m.orientation.z = q.z();
        m.orientation.w = q.w();
        if (m.orientation.w < 0)
        {
            m.orientation.x *= -1;
            m.orientation.y *= -1;
            m.orientation.z *= -1;
            m.orientation.w *= -1;
        }
        return m;
    }

    /*!
     * \brief Convert Eigen::Affine3d pose into an geometry_msgs::msg::Point translation element.
     * \param e is Eigen::Affine3d defining pose.
     * \return  geometry_msgs::msg::Point translation element.
     */
    template <>
    inline PointMsg Convert<Eigen::Affine3d, PointMsg>(Eigen::Affine3d pose)
    {
        PoseMsg msg = Convert<Eigen::Affine3d, PoseMsg>(pose);
        return msg.position;
    }

    // URDF

    /*!
     * \brief Convert urdf::Vector3 into an Eigen vector.
     * \param v is a urdf::Vector3.
     * \return  Eigen::Vector3d vector.
     */
    template <>
    inline Eigen::Vector3d Convert<urdf::Vector3, Eigen::Vector3d>(urdf::Vector3 v)
    {
        return Eigen::Vector3d(v.x, v.y, v.z);
    }

    template <>
    inline Eigen::Vector3d Convert<QuaternionMsg, Eigen::Vector3d>(QuaternionMsg v)
    {
        double r_w = v.w;
        double r_x = v.x;
        double r_y = v.y;
        double r_z = v.z;

        // Get Euler Angle(case zyx) from quaternions
        double sinr = 2.0 * (r_y * r_z + r_w * r_x);
        double cosr = 1.0 - 2.0 * (r_x * r_x + r_y * r_y);
        double roll = std::atan2(sinr, cosr) * 180.0 / M_PI;

        double sinp = 2.0 * (r_w * r_y - r_x * r_z);
        double pitch;
        if (std::fabs(sinp) >= 1)
        {
            pitch = std::copysign(M_PI / 2, sinp) * 180.0 / M_PI;
        }
        else
        {
            pitch = std::asin(sinp) * 180.0 / M_PI;
        }
        double siny = 2.0 * (r_x * r_y + r_w * r_z);
        double cosy = 1.0 - 2.0 * (r_y * r_y + r_z * r_z);
        double yaw = std::atan2(siny, cosy) * 180.0 / M_PI;

        return Eigen::Vector3d(roll, pitch, yaw);
    }

    /*!
     * \brief Convert urdf::Pose into an Eigen affine3d.
     * \param pose is a urdf::Pose.
     * \return  Eigen::Affine3d pose.
     */
    template <>
    inline Eigen::Affine3d Convert<urdf::Pose, Eigen::Affine3d>(urdf::Pose pose)
    {
        // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
        Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
        Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q);
        return af;
    }
#ifdef CRCL

    // CRCL
    /*!
     * \brief Convert array of std::vector<double> doubles into an JointState position, but blanking velcity, and effort.
     * \param src is a std::vector of doubles defining the value for each joint.
     * \return  sensor_msgs::msg::JointState definition.
     */
    template <>
    inline JointState Convert<std::vector<double>, JointState>(std::vector<double> src)
    {
        JointState joints;
        joints.position = src;
        joints.velocity.assign(src.size(), 0.0);
        joints.effort.assign(src.size(), 0.0);
        return joints;
    }

    //////////////////////////////////////////////////////////////////////////////////////////

    template <>
    inline Eigen::VectorXd Convert<std::vector<double>, Eigen::VectorXd>(std::vector<double> v)
    {
        Eigen::VectorXd p(v.size());
        for (size_t i = 0; i < v.size(); i++)
            p(i) = v[i];
        return p;
    }

    template <>
    inline std::vector<double> Convert<Eigen::VectorXd, std::vector<double>>(Eigen::VectorXd ev)
    {
        std::vector<double> v;
        for (int i = 0; i < ev.size(); i++)
            v.push_back(ev(i));
        return v;
    }
#endif
}
#endif