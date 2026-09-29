#pragma once

#include "types/command.hpp"
#include "types/joint.hpp"

namespace horkin
{
    class IJointDriver
    {
        public:
            virtual ~IJointDriver() = default;

            virtual void Enable() = 0;
            virtual void Disable() = 0;

            virtual void Read(JointState& out) = 0;

            virtual void Write(const JointCommand& cmd) = 0;
    };
}