#include "kine/robot_model.hpp"
#include "types/jacobian.hpp"
#include "types/joint.hpp"
#include "types/pose.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");

    horkin::JointVec q{};
    const horkin::Pose p0 = robot.fk(q);

    q[2] = 0.144;
    const horkin::Pose p1 = robot.fk(q);

    const double dx = p1.position.x - p0.position.x;
    const double dy = p1.position.y - p0.position.y;
    const double dz = p1.position.z - p0.position.z;
    const double dp = std::hypot(dx, std::hypot(dy, dz));
    if (dp < 0.1)
    {
        std::fprintf(stderr, "J3=0.144 m should move TCP, |dp|=%.4f\n", dp);
        return 1;
    }

    std::printf("test_fk ok  |dp|=%.4f m\n", dp);
    std::printf("  p0  x=%.4f y=%.4f z=%.4f\n", p0.position.x, p0.position.y, p0.position.z);
    std::printf("  p1  x=%.4f y=%.4f z=%.4f\n", p1.position.x, p1.position.y, p1.position.z);
    
    horkin::JointVec qj{};
    const horkin::Jacobian J = robot.jacobian(qj);

    const double dq = 1e-4;
    qj[2] = dq;
    const horkin::Pose p_plus = robot.fk(qj);

    const double vx = (p_plus.position.x - p0.position.x) / dq;
    const double vy = (p_plus.position.y - p0.position.y) / dq;
    const double vz = (p_plus.position.z - p0.position.z) / dq;

    const double e = std::hypot(vx - J[2][0], vy - J[2][1], vz - J[2][2]);
    if (e > 1e-3)
    {
        std::fprintf(stderr, "jacobian col 2 vs fd |e|=%.4e/n", e);
        return 1;
    }

    std::printf("test_jacobian ok  |e|=%.4f m\n", e);
    std::printf("  dp  x=%.4f y=%.4f z=%.4f\n", J[2][0], J[2][1], J[2][2]);
    
    return 0;
}