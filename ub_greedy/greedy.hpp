#pragma once

#include "gap/lib/solution.hpp"
#include "gap/lib/desirability.hpp"

namespace gap
{

std::vector<std::pair<ItemIdx, AgentIdx>> sol_greedy_init(const Solution& sol, const Desirability& f);
void sol_greedy(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output sol_greedy(const Instance& ins, const Desirability& f, Info info = Info());

std::vector<std::vector<AgentIdx>> sol_greedyregret_init(const Instance& ins, const Desirability& f);
void sol_greedyregret(Solution& sol, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output sol_greedyregret(const Instance& ins, const Desirability& f, Info info = Info());

void sol_mthg(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output sol_mthg(const Instance& ins, const Desirability& f, Info info = Info());

void sol_mthgregret(Solution& ins, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output sol_mthgregret(const Instance& ins, const Desirability& f, Info info = Info());

}

