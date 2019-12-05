#pragma once

#if LOCALSOLVER_FOUND

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
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

LocalSolverOutput sol_localsolver(const Instance& ins, LocalSolverOptionalParameters p = {});

}

#endif

