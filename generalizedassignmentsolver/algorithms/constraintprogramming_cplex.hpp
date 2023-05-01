#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ConstraintProgrammingCplexOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();
};

Output constraintprogramming_cplex(
        const Instance& instance,
        ConstraintProgrammingCplexOptionalParameters parameters = {});

}

#endif

