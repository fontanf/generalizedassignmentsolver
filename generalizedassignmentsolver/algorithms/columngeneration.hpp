#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "knapsacksolver/algorithms/minknap.hpp"
#include "knapsacksolver/algorithms/bellman.hpp"

namespace generalizedassignmentsolver
{

struct ColGenOptionalParameters
{
    Info info = Info();

    std::string lp_solver = "clp"; // "clp", "cplex"
    std::vector<std::vector<std::vector<ItemIdx>>>* columns = NULL;
    std::vector<std::vector<int>>* fixed_alt = NULL; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
    std::vector<int>* fixed_agents = NULL; // 0: unfixed, 1: fixed.
};

struct ColGenOutput: Output
{
    ColGenOutput(const Instance& instance, Info& info): Output(instance, info) { }
    ColGenOutput& algorithm_end(Info& info);

    /** Left empty if parameters.columns != NULL. */
    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<std::pair<AgentIdx, ColIdx>> column_indices;
    std::vector<double> solution;
    std::vector<std::vector<double>> x;
    Counter it = 0;
    Counter added_column_number = 0;
};

ColGenOutput columngeneration(const Instance& instance, ColGenOptionalParameters parameters = {});

}

