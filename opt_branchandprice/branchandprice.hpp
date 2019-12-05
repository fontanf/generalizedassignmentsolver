#pragma once

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

struct BranchAndPriceOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"

    BranchAndPriceOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("solver")) != args.end()) solver = it->second;
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

