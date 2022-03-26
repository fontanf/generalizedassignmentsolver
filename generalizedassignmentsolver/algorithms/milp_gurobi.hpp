#pragma once

#if GUROBI_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpGurobiOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
    bool only_linear_relaxation = false;
};

struct MilpGurobiOutput: Output
{
    MilpGurobiOutput(
            const Instance& instance,
            Info& info):
        Output(instance, info) { }

    MilpGurobiOutput& algorithm_end(Info& info);

    std::vector<std::vector<double>> x;
};

MilpGurobiOutput milp_gurobi(
        const Instance& instance,
        MilpGurobiOptionalParameters parameters = {});

}

#endif

