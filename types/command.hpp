#pragma once

#include "types/joint.hpp"

namespace horkin
{
    enum class JointCyclicMode
    {
        MIT,
        CSP,
        CSV,
        CST
    };

    struct JointCommand
    {
        JointVec q{};
        JointVec dq{};
        JointVec tau_ff{};
        JointVec kp{};
        JointVec kd{};
    };
}