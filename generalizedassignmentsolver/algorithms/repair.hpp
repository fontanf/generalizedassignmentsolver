#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#if COINOR_FOUND
#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#endif

namespace generalizedassignmentsolver
{

Output repairgreedy(const Instance& ins, Info info = Info());
Output repaircombrelax(const Instance& ins, Info info = Info());
#if COINOR_FOUND
Output repairlinrelax_clp(const Instance& ins, const LinRelaxClpOutput& linrelax_output, Info info = Info());
#endif

}

