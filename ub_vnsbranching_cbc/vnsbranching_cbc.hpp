#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_vnsbranching_cbc(const Instance& ins, std::mt19937_64& gen, Info info = Info());

}

#endif

