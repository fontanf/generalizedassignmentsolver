#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#if COINOR_FOUND
#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#endif

namespace gap
{

Solution sol_repairgreedy(const Instance& ins, Info info = Info());
Solution sol_repaircombrelax(const Instance& ins, Info info = Info());
#if COINOR_FOUND
Solution sol_repairlinrelax_clp(const Instance& ins, const LinRelaxClpOutput& linrelax_output, Info info = Info());
#endif

}

