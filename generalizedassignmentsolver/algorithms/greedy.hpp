#pragma once

#include "generalizedassignmentsolver/solution.hpp"
#include "generalizedassignmentsolver/desirability.hpp"

namespace generalizedassignmentsolver
{

std::vector<std::pair<ItemIdx, AgentIdx>> greedy_init(const Solution& solution, const Desirability& f);
void greedy(Solution& solution, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output greedy(const Instance& instance, const Desirability& f, Info info = Info());

std::vector<std::vector<AgentIdx>> greedyregret_init(const Instance& instance, const Desirability& f);
void greedyregret(Solution& solution, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output greedyregret(const Instance& instance, const Desirability& f, Info info = Info());

void mthg(Solution& solution, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt);
Output mthg(const Instance& instance, const Desirability& f, Info info = Info());

void mthgregret(Solution& instance, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt);
Output mthgregret(const Instance& instance, const Desirability& f, Info info = Info());

}

