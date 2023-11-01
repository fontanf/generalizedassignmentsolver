#pragma once

#if VOLUME_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////// lagrelax_assignment_volume //////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxAssignmentVolumeOutput: Output
{
    LagRelaxAssignmentVolumeOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_items() */
    std::vector<double> multipliers;
};

LagRelaxAssignmentVolumeOutput lagrelax_assignment_volume(
        const Instance& instance,
        optimizationtools::Info info = optimizationtools::Info());

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_knapsack_volume ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxKnapsackVolumeOutput: Output
{
    LagRelaxKnapsackVolumeOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_agents() */
    std::vector<double> multipliers;
};

LagRelaxKnapsackVolumeOutput lagrelax_knapsack_volume(
        const Instance& instance,
        optimizationtools::Info info = optimizationtools::Info());

}

#endif
