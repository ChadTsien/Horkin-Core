#pragma once

#include "kine/robot_model.hpp"
#include "types/joint.hpp"
#include "types/pose.hpp"

#include <array>

namespace horkin 
{
    // 坐标系： [vx, vy, vz, wx, wy, wz]
    // 末端一拍要走的六维空间速度  J dq = twist  描述的是小偏移增量
    using Twist = std::array<double, 6>;

    struct IkParams
    {
        double damping{1e-2};
        double ori_length{1.0};
    };

    struct IkPoseParams
    {
        IkParams step{};        // 每拍 ik_step 的参数：阻尼 λ、姿态权重 ori_length（m/rad）
        double pos_tol{1e-4};   // 位置收敛阈值，米
        double ori_tol{1e-3};   // 姿态收敛阈值，弧度
        double max_lin{0.02};   // 每拍送进 ik_step 的最大平移，米
        double max_ang{0.05};   // 每拍送进 ik_step 的最大转动，弧度
        int max_iter{40};       // 最大迭代次数，超过则失败
    };

    // 一次迭代，多用在力控实时性高的系统
    JointVec ik_step(const RobotModel& model, const JointVec& q,
                     const Twist& twist, const IkParams& params = {});
    
    // 多次迭代求取精确解，多用在路径规划
    bool ik_pose(const RobotModel& model, JointVec& q, const Pose& target,
                 const IkPoseParams& params = {});
}