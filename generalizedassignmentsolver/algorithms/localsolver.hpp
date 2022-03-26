#pragma once

#if LOCALSOLVER_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSolverOptionalParameters
{
    Info info = Info();
};

struct LocalSolverOutput: Output
{
    LocalSolverOutput(
            const Instance& instance,
            Info& info): Output(instance, info) { }

    LocalSolverOutput& algorithm_end(Info& info);
};

LocalSolverOutput localsolver(
        const Instance& instance,
        LocalSolverOptionalParameters p = {});

}

#endif

