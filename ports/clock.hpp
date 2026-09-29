#pragma once

#include "types/time.hpp"

namespace horkin
{
    class IClock
    {
        public:
            virtual ~IClock() = default;

            virtual TimeNs Now() const = 0;
    };
}