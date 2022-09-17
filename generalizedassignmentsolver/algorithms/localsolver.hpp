#pragma once

#if LOCALSOLVER_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSolverOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();
};

struct LocalSolverOutput: Output
{
    LocalSolverOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    LocalSolverOutput& algorithm_end(
            optimizationtools::Info& info);
};

LocalSolverOutput localsolver(
        const Instance& instance,
        LocalSolverOptionalParameters p = {});

}

#endif

