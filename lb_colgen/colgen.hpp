#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "knapsack/opt_minknap/minknap.hpp"
#include "knapsack/opt_bellman/bellman.hpp"

namespace gap
{

struct ColGenData
{
    const Instance& ins;
    Cost lb = 0;
    std::vector<std::vector<std::vector<uint8_t>>>& columns; // y[i][k][j]
    std::vector<AltIdx>& fixed_alt; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex" or "gurobi"

    ColGenData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("s")) != args.end()) solver = it->second;
        return *this;
    }
};
void lb_colgen(ColGenData d);

}

