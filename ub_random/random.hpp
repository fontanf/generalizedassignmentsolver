#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_random(const Instance& ins, std::default_random_engine& gen, Info info = Info());

}

