#pragma once

#if CPLEX_FOUND

#include "gap/lib/solution.hpp"

#include "knapsack/opt_minknap/minknap.hpp"
#include "knapsack/opt_bellman/bellman.hpp"

namespace gap
{

struct ColGenCplexOptionalParameters
{
    Info info = Info();

    std::vector<std::vector<std::vector<ItemIdx>>>* columns = NULL;
    std::vector<int>* fixed_alt = NULL; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
    std::vector<int>* fixed_agents = NULL; // 0: unfixed, 1: fixed.
};

struct ColGenCplexOutput: Output
{
    ColGenCplexOutput(const Instance& ins, Info& info): Output(ins, info) { }
    ColGenCplexOutput& algorithm_end(Info& info);

    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<std::pair<AgentIdx, ColIdx>> column_indices;
    std::vector<double> solution;
    std::vector<double> x;
    Cpt it = 0;
    Cpt added_column_number = 0;
};

ColGenCplexOutput lb_colgen_cplex(const Instance& ins, ColGenCplexOptionalParameters p = {});

}

#endif

