#pragma once

#if CPLEX_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Output sol_vnsbranching_cplex(const Instance& ins, std::mt19937_64& gen, Info info = Info());

}

#endif

