#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_lssimple(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());

}

