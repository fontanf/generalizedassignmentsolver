#pragma once

#include "generalizedassignmentsolver/solution.hpp"
#include "generalizedassignmentsolver/desirability.hpp"

namespace generalizedassignmentsolver
{

// These are the 4 main functions of the package.

Output greedy(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info = optimizationtools::Info());

Output greedy_regret(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info = optimizationtools::Info());

Output mthg(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info = optimizationtools::Info());

Output mthg_regret(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info = optimizationtools::Info());

// These additional functions can be used to fill a partial solutions.
//
// auto aternatives = greedy_init(instance, f);
// greedy(solution_1, alternatives);
// mthg(solution_2, alternatives);
//
// auto agents = greedyregret_init(instance, f);
// greedyregret(solution_1, agents)
// mthgregret(solution_2, agents)

std::vector<std::pair<ItemIdx, AgentIdx>> greedy_init(
        const Instance& instance,
        const Desirability& f);

void greedy(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives);

void mthg(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives);


std::vector<std::vector<AgentIdx>> greedy_regret_init(
        const Instance& instance,
        const Desirability& f);

void greedy_regret(
        Solution& solution,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives = {});

void mthg_regret(
        Solution& instance,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives = {});

}

