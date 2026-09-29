// 机械臂关节类型
#pragma once

#include <array>
#include <cstddef>

namespace horkin
{
    enum class JointKind
    {
        Revolute,
        Prismatic,
        Fixed
    };

    inline constexpr std::size_t kNJoints = 6;

    inline constexpr std::array<JointKind, kNJoints> kJointKind = {
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Prismatic,
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Revolute,
    };
}