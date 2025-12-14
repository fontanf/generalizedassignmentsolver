#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
///////////////////// lagrangian_relaxation_assignment_dlib ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationAssignmentDlibParameters: Parameters
{
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
        std::vector<int>* initial_multipliers = NULL,
        std::vector<std::vector<int>>* fixed_alt = NULL,
        const LagrangianRelaxationAssignmentDlibParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
/////////////////////// lagrangian_relaxation_knapsack_dlib ////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationKnapsackDlibParameters: Parameters
{
};

struct LagrangianRelaxationKnapsackDlibOutput: Output
{
    LagrangianRelaxationKnapsackDlibOutput(
            const Instance& instance):
        Output(instance) { }

    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationKnapsackDlibOutput lagrangian_relaxation_knapsack_dlib(
        const Instance& instance,
        std::vector<int>* initial_multipliers = NULL,
        std::vector<std::vector<int>>* fixed_alt = NULL,
        const Parameters& parameters = {});

}
