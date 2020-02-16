#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
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

ConstraintProgrammingCplexOutput constraintprogramming_cplex(const Instance& ins, ConstraintProgrammingCplexOptionalParameters p = {});

}

#endif

