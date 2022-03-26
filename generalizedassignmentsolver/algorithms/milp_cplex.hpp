#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpCplexOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
    bool only_linear_relaxation = false;
};

struct MilpCplexOutput: Output
{
    MilpCplexOutput(
            const Instance& instance,
            Info& info):
        Output(instance, info) { }

    MilpCplexOutput& algorithm_end(Info& info);

    std::vector<std::vector<double>> x;
};

MilpCplexOutput milp_cplex(
        const Instance& instance,
        MilpCplexOptionalParameters parameters = {});

}

#endif

