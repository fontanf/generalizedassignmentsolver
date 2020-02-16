#pragma once

#include "generalizedassignmentsolver/solution.hpp"
#include "generalizedassignmentsolver/desirability.hpp"

namespace generalizedassignmentsolver
{

std::vector<std::pair<ItemIdx, AgentIdx>> greedy_init(const Solution& sol, const Desirability& f);
void greedy(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output greedy(const Instance& ins, const Desirability& f, Info info = Info());

std::vector<std::vector<AgentIdx>> greedyregret_init(const Instance& ins, const Desirability& f);
void greedyregret(Solution& sol, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output greedyregret(const Instance& ins, const Desirability& f, Info info = Info());

void mthg(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output mthg(const Instance& ins, const Desirability& f, Info info = Info());

void mthgregret(Solution& ins, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output mthgregret(const Instance& ins, const Desirability& f, Info info = Info());

}

