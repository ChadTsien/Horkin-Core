#pragma once

namespace horkin
{
    struct Vec3
    {
        double x{};
        double y{};
        double z{};
    };

    struct Rotation
    {
        Vec3 x{};  // row 0: R00 R01 R02
        Vec3 y{};  // row 1: R10 R11 R12
        Vec3 z{};  // row 2: R20 R21 R22
    };

    struct Pose
    {
        Vec3 position{};
        Rotation rotation{ {1, 0, 0}, 
                           {0, 1, 0}, 
                           {0, 0, 1} };
    };
}