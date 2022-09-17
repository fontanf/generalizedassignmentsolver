#pragma once

#if GUROBI_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpGurobiOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    const Solution* initial_solution = NULL;
    bool only_linear_relaxation = false;
};

struct MilpGurobiOutput: Output
{
    MilpGurobiOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    MilpGurobiOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x;
};

MilpGurobiOutput milp_gurobi(
        const Instance& instance,
        MilpGurobiOptionalParameters parameters = {});

}

#endif

