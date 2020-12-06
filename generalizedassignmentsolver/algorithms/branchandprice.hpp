#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

namespace generalizedassignmentsolver
{

struct BranchAndPriceOptionalParameters
{
    Info info = Info();

    std::string lp_solver = "clp"; // "clp", "cplex"
    std::string tree_search_algorithm = "bfs"; // "dfs", "bfs", "lds"
    std::string branching_rule = "most-fractional"; // "most-fractional", "most-integer"
};

struct BranchAndPriceOutput: Output
{
    BranchAndPriceOutput(const Instance& instance, Info& info): Output(instance, info) { }
    BranchAndPriceOutput& algorithm_end(Info& info);

    Counter node_number = 0;
    ColIdx column_number = 0;
};

BranchAndPriceOutput branchandprice(
        const Instance& instance, BranchAndPriceOptionalParameters parameters = {});

}

