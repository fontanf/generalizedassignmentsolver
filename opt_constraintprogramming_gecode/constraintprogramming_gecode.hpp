#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct ConstraintProgrammingGecodeData
{
    const Instance& ins;
    Solution& sol;
    Info info = Info();
};

Solution sopt_constraintprogramming_gecode(ConstraintProgrammingGecodeData d);

}

