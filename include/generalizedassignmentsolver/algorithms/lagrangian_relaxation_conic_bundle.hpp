#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
///////////////////// lagrangian_relaxation_assignment_conic_bundle ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationAssignmentConicBundleParameters: Parameters
{
};

struct LagrangianRelaxationAssignmentConicBundleOutput: Output
{
    LagrangianRelaxationAssignmentConicBundleOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationAssignmentConicBundleOutput lagrangian_relaxation_assignment_conic_bundle(
        const Instance& instance,
        const LagrangianRelaxationAssignmentConicBundleParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
/////////////////////// lagrangian_relaxation_knapsack_conic_bundle ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationKnapsackConicBundleOutput: Output
{
    LagrangianRelaxationKnapsackConicBundleOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number() */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_agents() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationKnapsackConicBundleOutput lagrangian_relaxation_knapsack_conic_bundle(
        const Instance& instance,
        const Parameters& parameters = {});

}
