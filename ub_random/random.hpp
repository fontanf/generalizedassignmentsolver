#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Output sol_random_infeasible(const Instance& ins, std::mt19937_64& gen, Info info = Info());
Output sol_random(const Instance& ins, std::mt19937_64& gen, Info info = Info());

}

