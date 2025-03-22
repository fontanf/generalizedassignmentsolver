#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
///////////////////// lagrangian_relaxation_assignment_dlib ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationAssignmentDlibParameters: Parameters
{
    std::vector<int>* initial_multipliers = NULL;

    /** -1: unfixed, 0: fixed to 0, 1: fixed to 1. */
    std::vector<std::vector<int>>* fixed_alt = NULL;
};

struct LagrangianRelaxationAssignmentDlibOutput: Output
{
    LagrangianRelaxationAssignmentDlibOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationAssignmentDlibOutput lagrangian_relaxation_assignment_dlib(
        const Instance& instance,
        const LagrangianRelaxationAssignmentDlibParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
/////////////////////// lagrangian_relaxation_knapsack_dlib ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationKnapsackDlibOutput: Output
{
    LagrangianRelaxationKnapsackDlibOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number() */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_agents() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationKnapsackDlibOutput lagrangian_relaxation_knapsack_dlib(
        const Instance& instance,
        const Parameters& parameters = {});

}
