#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_vdns_simple(const Instance& ins, Solution& sol_best, std::mt19937_64& gen, Info info = Info());

}

#endif

