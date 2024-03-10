#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

// These are the 4 main functions of the package.

struct GreedyParameters: Parameters
{
    std::string desirability = "cij";


    virtual int format_width() const override { return 26; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Desirability: " << desirability << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                {"Desirability", desirability},
                });
        return json;
    }
};

const Output greedy(
        const Instance& instance,
        const GreedyParameters& parameters = {});

const Output greedy_regret(
        const Instance& instance,
        const GreedyParameters& parameters = {});

const Output mthg(
        const Instance& instance,
        const GreedyParameters& parameters = {});

const Output mthg_regret(
        const Instance& instance,
        const GreedyParameters& parameters = {});

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
        const std::vector<std::vector<double>>& desirability);

void greedy(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives);

void mthg(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives);


std::vector<std::vector<AgentIdx>> greedy_regret_init(
        const Instance& instance,
        const std::vector<std::vector<double>>& desirability);

void greedy_regret(
        Solution& solution,
        const std::vector<std::vector<double>>& desirability,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives = {});

void mthg_regret(
        Solution& instance,
        const std::vector<std::vector<double>>& desirability,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives = {});

}
