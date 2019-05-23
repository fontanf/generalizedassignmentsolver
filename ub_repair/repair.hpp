#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

namespace gap
{

Solution sol_repairlinrelax(const Instance& ins, const LinRelaxClpOutput& linrelax_output, Info info = Info());

}

