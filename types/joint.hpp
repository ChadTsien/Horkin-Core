// 关节数据类型
#pragma once

#include "joint_layout.hpp"
#include "types/joint_layout.hpp"

namespace horkin
{
    // 逻辑关节向量，长度等于关节数量
    using JointVec = std::array<double, kNJoints>;

    struct JointState
    {
        JointVec q{};
        JointVec dq{};
        JointVec tau{};
        bool valid{false};
    };
}