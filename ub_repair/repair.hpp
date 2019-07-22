#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

namespace gap
{

Solution sol_repairgreedy(const Instance& ins, Info info = Info());
Solution sol_repaircombrelax(const Instance& ins, Info info = Info());
Solution sol_repairlinrelax(const Instance& ins, const LinRelaxClpOutput& linrelax_output, Info info = Info());

}

#endif

