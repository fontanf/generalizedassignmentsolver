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
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_items() */
    std::vector<double> multipliers;
};

const LagRelaxAssignmentVolumeOutput lagrelax_assignment_volume(
        const Instance& instance,
        const Parameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_knapsack_volume ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxKnapsackVolumeOutput: Output
{
    LagRelaxKnapsackVolumeOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_agents() */
    std::vector<double> multipliers;
};

const LagRelaxKnapsackVolumeOutput lagrelax_knapsack_volume(
        const Instance& instance,
        const Parameters& parameters = {});

}

#endif
