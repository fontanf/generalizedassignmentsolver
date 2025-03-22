#pragma once

#if VOLUME_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
//////////////////// lagrangian_relaxation_assignment_volume ///////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationAssignmentVolumeOutput: Output
{
    LagrangianRelaxationAssignmentVolumeOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationAssignmentVolumeOutput lagrangian_relaxation_assignment_volume(
        const Instance& instance,
        const Parameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
///////////////////// lagrangian_relaxation_knapsack_volume ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationKnapsackVolumeOutput: Output
{
    LagrangianRelaxationKnapsackVolumeOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;

    /** vector of size ins.number_of_agents() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationKnapsackVolumeOutput lagrangian_relaxation_knapsack_volume(
        const Instance& instance,
        const Parameters& parameters = {});

}

#endif
