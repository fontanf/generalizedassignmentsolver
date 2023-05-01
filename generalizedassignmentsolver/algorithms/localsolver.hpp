#pragma once

#if LOCALSOLVER_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSolverOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();
};

Output localsolver(
        const Instance& instance,
        LocalSolverOptionalParameters p = {});

}

#endif

