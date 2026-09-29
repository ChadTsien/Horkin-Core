#pragma once

#include "types/wrench.hpp"

namespace horkin 
{
    class IForceSensor
    {
        public:
            virtual ~IForceSensor() = default;

            virtual void Read(WrenchSample& out) = 0;
    };
}