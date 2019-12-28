#pragma once

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

struct BranchAndPriceOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"

    BranchAndPriceOptionalParameters& set_params(const std::vector<std::string>& argv)
    {
        for (auto it = argv.begin() + 1; it != argv.end(); ++it) {
            if (*it == "solver") { solver = *(++it); }
        }
        return *this;
    }
};

struct BranchAndPriceOutput: Output
{
    BranchAndPriceOutput(const Instance& ins, Info& info): Output(ins, info) { }
    BranchAndPriceOutput& algorithm_end(Info& info);

    Cpt node_number = 0;
};

BranchAndPriceOutput sopt_branchandprice_dfs(const Instance& ins, BranchAndPriceOptionalParameters p = {});

BranchAndPriceOutput sopt_branchandprice_astar(const Instance& ins, BranchAndPriceOptionalParameters p = {});

}

