#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct ConstraintProgrammingCplexData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution sopt_constraintprogramming_cplex(ConstraintProgrammingCplexData d);

}

