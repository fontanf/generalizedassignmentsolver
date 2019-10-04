#pragma once

#if GUROBI_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct BranchAndCutGurobiOptionalParameters
{
    Info info = Info();
};

struct BranchAndCutGurobiOutput: Output
{
    BranchAndCutGurobiOutput(const Instance& ins, Info& info): Output(ins, info) { }
};

BranchAndCutGurobiOutput sopt_branchandcut_gurobi(const Instance& ins, BranchAndCutGurobiOptionalParameters p = {});

}

#endif

