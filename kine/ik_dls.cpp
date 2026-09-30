#include "kine/ik_dls.hpp"
#include "kine/robot_model.hpp"
#include "types/jacobian.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"
#include "types/pose.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace horkin
{
    namespace
    {
        constexpr int kTwist = 6;  // 空间速度，六维：vx,vy,vz,wx,wy,wz

        // 外部 Pose 类型与 Pinocchio 接口，为了在外不暴露 Pinocchio
        pinocchio::SE3 to_se3(const Pose& pose)
        {
            Eigen::Matrix3d R;
            R << pose.rotation.x.x, pose.rotation.x.y, pose.rotation.x.z,
                 pose.rotation.y.x, pose.rotation.y.y, pose.rotation.y.z,
                 pose.rotation.z.x, pose.rotation.z.y, pose.rotation.z.z;
            
            return pinocchio::SE3(R, Eigen::Vector3d(pose.position.x,
                                                            pose.position.y,
                                                            pose.position.z));
        }

        // 世界系下原点线速度(原点位移差) + 世界角速度(旋转角度差)
        Twist pose_error(const Pose& current, const Pose& target)
        {
            const pinocchio::SE3 Mc = to_se3(current);
            const pinocchio::SE3 Md = to_se3(target);
            const Eigen::Vector3d dp = Md.translation() - Mc.translation();
            const Eigen::Vector3d w = pinocchio::log3(Md.rotation() * Mc.rotation().transpose());
            return {dp.x(), dp.y(), dp.z(), w.x(), w.y(), w.z()};
        }

        Eigen::Matrix<double, kTwist, 1> to_eigen(const Twist& twist)
        {
            Eigen::Matrix<double, kTwist, 1> v;
            for (int r = 0; r < kTwist; ++r)
            {
                v[r] = twist[static_cast<std::size_t>(r)];
            }
            return v;
        }

        void cap_twist(Twist& twist, double max_lin, double max_ang)
        {
            Eigen::Vector3d lin(twist[0], twist[1], twist[2]);
            Eigen::Vector3d ang(twist[3], twist[4], twist[5]);
            if (lin.norm() > max_lin)
            {
                lin *= max_lin / lin.norm();
            }
            if (ang.norm() > max_ang)
            {
                ang *= max_ang / ang.norm();
            }
            twist[0] = lin.x();
            twist[1] = lin.y();
            twist[2] = lin.z();
            twist[3] = ang.x();
            twist[4] = ang.y();
            twist[5] = ang.z();
        }

        void clamp_q(const RobotModel& model, JointVec& q)
        {
            const JointVec lower = model.q_lower();
            const JointVec upper = model.q_upper();
            for (std::size_t j = 0; j < kNJoints; ++j)
            {
                q[j] = std::clamp(q[j], lower[j], upper[j]);
            }
        }
    }

    JointVec ik_step(const RobotModel &model, const JointVec &q, 
                     const Twist &twist, const IkParams& params)
    {
        const Jacobian Jpod = model.jacobian(q);

        Eigen::Matrix<double, kTwist, Eigen::Dynamic> J(kTwist, static_cast<int>(kNJoints));
        for (int r = 0; r < kTwist; ++r)
        {
            for (std::size_t j = 0; j < kNJoints; ++j)
            {
                J(r, static_cast<int>(j)) = Jpod[static_cast<std::size_t>(r)][j];
            }
        }

        Eigen::Matrix<double, kTwist, 1> w;
        w << 1.0, 1.0, 1.0, params.ori_length, params.ori_length, params.ori_length;
        Eigen::Matrix<double, kTwist, Eigen::Dynamic> Js = w.asDiagonal() * J;

        Eigen::Matrix<double, Eigen::Dynamic, 1> s(static_cast<int>(kNJoints));
        for (std::size_t j = 0; j < kNJoints; ++j)
        {
            const double n = Js.col(static_cast<int>(j)).norm();
            s[static_cast<int>(j)] = (n < 1e-8) ? 0.0 : (1.0 / n);
            Js.col(static_cast<int>(j)) *= s[static_cast<int>(j)];
        }

        Eigen::Matrix<double, kTwist, kTwist> A = Js * Js.transpose();
        A.diagonal().array() += params.damping * params.damping;

        const Eigen::Matrix<double, kTwist, 1> b = w.cwiseProduct((to_eigen(twist)));
        const Eigen::Matrix<double, kTwist, 1> y = A.ldlt().solve(b);
        const Eigen::VectorXd u = Js.transpose() * y;

        JointVec dq{};
        for (std::size_t j = 0; j < kNJoints; ++j)
        {
            dq[j] = s[static_cast<int>(j)] * u[static_cast<int>(j)];
        }
        return dq;
    }

    bool ik_pose(const RobotModel &model, JointVec &q, const Pose &target, const IkPoseParams& params)
    {
        for (int iter = 0; iter < params.max_iter; ++iter)
        {
            Twist error = pose_error(model.fk(q), target);
            const Eigen::Vector3d lin(error[0], error[1], error[2]);
            const Eigen::Vector3d ang(error[3], error[4], error[5]);
            if (lin.norm() < params.pos_tol && ang.norm() < params.ori_tol)
            {
                return true;
            }

            cap_twist(error, params.max_lin, params.max_ang);
            const JointVec dq = ik_step(model, q, error, params.step);
            for (std::size_t j = 0; j < kNJoints; ++j)
            {
                q[j] += dq[j];
            }
            clamp_q(model, q);  
        }
        return false;
    }
}