#pragma once

#if COINOR_FOUND

#include "gap/lib/solution.hpp"

namespace gap
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
};

BranchAndPriceOutput sopt_branchandprice(const Instance& ins, BranchAndPriceOptionalParameters p = {});

}

#endif

