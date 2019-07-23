#pragma once

#if CPLEX_FOUND
#if DLIB_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_vlsn_mbp(const Instance& ins, Solution& sol, std::mt19937_64& gen, Info info = Info());

}

#endif
#endif

