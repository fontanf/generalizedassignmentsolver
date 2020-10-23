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
    ConstraintProgrammingCplexOutput(const Instance& instance, Info& info): Output(instance, info) { }
    ConstraintProgrammingCplexOutput& algorithm_end(Info& info);
};

ConstraintProgrammingCplexOutput constraintprogramming_cplex(
        const Instance& instance,
        ConstraintProgrammingCplexOptionalParameters parameters = {});

}

#endif

