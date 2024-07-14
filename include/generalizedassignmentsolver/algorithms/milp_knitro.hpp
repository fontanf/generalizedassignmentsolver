#pragma once

#if KNITRO_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct MilpKnitroParameters: Parameters
{
    const Solution* initial_solution = NULL;

    bool only_linear_relaxation = false;
};

struct MilpKnitroOutput: Output
{
    MilpKnitroOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;
};

const MilpKnitroOutput milp_knitro(
        const Instance& instance,
        const MilpKnitroParameters& parameters = {});

}

#endif
