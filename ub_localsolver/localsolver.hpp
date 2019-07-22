#pragma once

#if LOCALSOLVER_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct LocalSolverData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution ub_localsolver(LocalSolverData d);

}

#endif

