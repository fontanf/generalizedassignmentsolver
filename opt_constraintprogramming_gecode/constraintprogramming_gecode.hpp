#pragma once

#if GECODE_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct ConstraintProgrammingGecodeData
{
    const Instance& ins;
    Solution& sol;
    Cost& lb;
    Info info = Info();
};

Solution sopt_constraintprogramming_gecode(ConstraintProgrammingGecodeData d);

}

#endif

