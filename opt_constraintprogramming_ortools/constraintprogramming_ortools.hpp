#pragma once

#if ORTOOLS_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct ConstraintProgrammingOrtoolsData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution sopt_constraintprogramming_ortools(ConstraintProgrammingOrtoolsData d);

}

#endif

