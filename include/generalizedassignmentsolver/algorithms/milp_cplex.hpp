#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpCplexParameters: Parameters
{
    const Solution* initial_solution = NULL;

    bool only_linear_relaxation = false;
};

struct MilpCplexOutput: Output
{
    MilpCplexOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;
};

const MilpCplexOutput milp_cplex(
        const Instance& instance,
        const MilpCplexParameters& parameters = {});

}

#endif
