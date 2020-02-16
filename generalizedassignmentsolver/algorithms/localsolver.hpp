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
    LocalSolverOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LocalSolverOutput& algorithm_end(Info& info);
};

LocalSolverOutput localsolver(const Instance& ins, LocalSolverOptionalParameters p = {});

}

#endif

