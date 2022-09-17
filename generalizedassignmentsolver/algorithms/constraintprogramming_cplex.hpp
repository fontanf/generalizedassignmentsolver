#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ConstraintProgrammingCplexOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();
};

struct ConstraintProgrammingCplexOutput: Output
{
    ConstraintProgrammingCplexOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    ConstraintProgrammingCplexOutput& algorithm_end(
            optimizationtools::Info& info);
};

ConstraintProgrammingCplexOutput constraintprogramming_cplex(
        const Instance& instance,
        ConstraintProgrammingCplexOptionalParameters parameters = {});

}

#endif

