#pragma once

#include "types/time.hpp"
#include <array>

namespace horkin 
{
    // Generic 6-vector. Wrench layout: [fx, fy, fz, mx, my, mz]
    using Vector6 = std::array<double, 6>;

    // Force-torque. Units: N, N·m.
    struct Wrench {
        Vector6 ft{};
    };

    // One sensor-frame raw sample. 
    struct WrenchSample
    {
        Wrench wrench{};
        TimeNs t_acq{0};      // capture time, monotonic ns
        bool valid{false};    // If valid is false, ignore the other fields.
    };
}