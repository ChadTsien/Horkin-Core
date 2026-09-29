#include "kine/ik_dls.hpp"
#include "types/jacobian.hpp"
#include "types/joint_layout.hpp"
#include "types/pose.hpp"

#include <Eigen/src/Core/Matrix.h>
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
        constexpr int kTwist = 6;

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

        // 世界系下原点线速度 + 世界角速度
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
    }
}