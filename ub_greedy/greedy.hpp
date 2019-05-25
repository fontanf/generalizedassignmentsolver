#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"
#include "gap/lib/desirability.hpp"

namespace gap
{

std::vector<std::pair<ItemIdx, AgentIdx>> sol_greedy_init(const Solution& sol, const Desirability& f);
void sol_greedy(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Solution sol_greedy(const Instance& ins, const Desirability& f, Info info = Info());

void sol_greedyregret(Solution& sol, const Desirability& f);
Solution sol_greedyregret(const Instance& ins, const Desirability& f, Info info = Info());

void sol_mthg(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Solution sol_mthg(const Instance& ins, const Desirability& f, Info info = Info());

void sol_mthgregret(Solution& ins, const Desirability& f);
Solution sol_mthgregret(const Instance& ins, const Desirability& f, Info info = Info());

}

