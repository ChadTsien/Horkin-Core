// 雅可比数据类型模板
#pragma once

#include "types/joint_layout.hpp"
#include "types/wrench.hpp"

#include <array>

namespace horkin
{   
    // 雅可比矩阵：根据列向量，其含义: [vx, vy, vz, wx, wy, wz]
    using Jacobian = std::array<std::array<double, kNJoints>, 6>;
}