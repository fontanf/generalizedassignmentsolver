#pragma once

#if COINOR_FOUND

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

    LagRelaxAssignmentVolumeOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x;
    std::vector<double> multipliers; // vector of size ins.number_of_items()
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

    LagRelaxKnapsackVolumeOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x;
    std::vector<double> multipliers; // vector of size ins.number_of_agents()
};

LagRelaxKnapsackVolumeOutput lagrelax_knapsack_volume(
        const Instance& instance,
        optimizationtools::Info info = optimizationtools::Info());

}

#endif

