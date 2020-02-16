#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct BranchAndPriceOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"
};

struct BranchAndPriceOutput: Output
{
    BranchAndPriceOutput(const Instance& ins, Info& info): Output(ins, info) { }
    BranchAndPriceOutput& algorithm_end(Info& info);

    Counter node_number = 0;
};

BranchAndPriceOutput branchandprice_dfs(const Instance& ins, BranchAndPriceOptionalParameters p = {});

BranchAndPriceOutput branchandprice_astar(const Instance& ins, BranchAndPriceOptionalParameters p = {});

}

