#pragma once

#if GUROBI_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpGurobiParameters: Parameters
{
    const Solution* initial_solution = NULL;

    bool only_linear_relaxation = false;
};

struct MilpGurobiOutput: Output
{
    MilpGurobiOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;
};

const MilpGurobiOutput milp_gurobi(
        const Instance& instance,
        const MilpGurobiParameters& parameters = {});

}

#endif

