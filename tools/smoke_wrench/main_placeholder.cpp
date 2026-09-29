#include "ports/clock.hpp"
#include "ports/force_sensor.hpp"
#include "ports/joint_driver.hpp"

#include <cstdio>
#include <cinttypes>
#include <cstddef>

namespace 
{
    class StubForceSensor final : public horkin::IForceSensor
    {
        public:
            void Read(horkin::WrenchSample& out) override
            {
                out = {};
                out.valid = true;
                out.t_acq = 1'000'000'000; 
                out.wrench.ft = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
    };

    class StubClock final : public horkin::IClock
    {
        public:
            horkin::TimeNs Now() const override { return 1'000'000'000; }
    };

    class StubJointDriver final : public horkin::IJointDriver
    {
        public:
            void Enable() override {}
            void Disable() override {}

            void Read(horkin::JointState& out) override
            {
                out = {};
                out.valid = true;
                for (std::size_t i = 0; i < horkin::kNJoints; ++i)
                {
                    if (horkin::kJointKind[i] == horkin::JointKind::Prismatic)
                    {
                        out.q[i] = 0.144;
                    }
                }
            }

            void Write(const horkin::JointCommand&) override {}
    };
}

int main()
{
    StubForceSensor sensor;
    horkin::WrenchSample sample;
    sensor.Read(sample);

    if (!sample.valid)
    {
        std::fprintf(stderr, "smoke_wench: invalid sample\n");
        return 1;
    }

    const horkin::Vector6& w = sample.wrench.ft;
    std::printf(
        "t_acq=%" PRIu64 " ns  wrench=[%.3f %.3f %.3f  %.3f %.3f %.3f]  (N, N·m)\n",
        sample.t_acq, w[0], w[1], w[2], w[3], w[4], w[5]);

    StubClock clock;
    StubJointDriver joints;
    horkin::JointState js;
    joints.Read(js);
    if (!js.valid)
    {
        std::fprintf(stderr, "smoke_wrench: invalid joint sample\n");
        return 1;
    }

    std::printf("now=%" PRIu64 " ns  n_joints=%zu\n",
        clock.Now(), horkin::kNJoints);

    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        const char* kind = 
            (horkin::kJointKind[i] == horkin::JointKind::Prismatic)
                ? "prismatic(m)"
                : "revolute(rad)";
        std::printf("  q[%zu]=%.3f  %s\n", i, js.q[i], kind);
    }
    return 0;
}
