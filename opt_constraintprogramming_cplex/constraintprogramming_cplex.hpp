#pragma once

#if CPLEX_FOUND

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

struct ConstraintProgrammingCplexOptionalParameters
{
    Info info = Info();
};

struct ConstraintProgrammingCplexOutput: Output
{
    ConstraintProgrammingCplexOutput(const Instance& ins, Info& info): Output(ins, info) { }
    ConstraintProgrammingCplexOutput& algorithm_end(Info& info);
};

ConstraintProgrammingCplexOutput sopt_constraintprogramming_cplex(const Instance& ins, ConstraintProgrammingCplexOptionalParameters p = {});

}

#endif

