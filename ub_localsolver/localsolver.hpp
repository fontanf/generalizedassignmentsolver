#pragma once

#if LOCALSOLVER_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct LocalSolverOptionalParameters
{
    Info info = Info();
};

struct LocalSolverOutput: Output
{
    LocalSolverOutput(const Instance& ins, Info& info): Output(ins, info) { }
};

LocalSolverOutput sol_localsolver(const Instance& ins, LocalSolverOptionalParameters p = {});

}

#endif

