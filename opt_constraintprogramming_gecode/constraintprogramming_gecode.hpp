#pragma once

#if GECODE_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct ConstraintProgrammingGecodeOptionalParameters
{
    Info info = Info();
};

struct ConstraintProgrammingGecodeOutput: Output
{
    ConstraintProgrammingGecodeOutput(const Instance& ins, Info& info): Output(ins, info) { }
};

ConstraintProgrammingGecodeOutput sopt_constraintprogramming_gecode(const Instance& ins, ConstraintProgrammingGecodeOptionalParameters p = {});

}

#endif

