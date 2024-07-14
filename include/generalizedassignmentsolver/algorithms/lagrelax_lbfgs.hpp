#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_assignment_lbfgs //////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxAssignmentLbfgsParameters: Parameters
{
    std::vector<int>* initial_multipliers = NULL;

    /** -1: unfixed, 0: fixed to 0, 1: fixed to 1. */
    std::vector<std::vector<int>>* fixed_alt = NULL;
};

struct LagRelaxAssignmentLbfgsOutput: Output
{
    LagRelaxAssignmentLbfgsOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagRelaxAssignmentLbfgsOutput lagrelax_assignment_lbfgs(
        const Instance& instance,
        const LagRelaxAssignmentLbfgsParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// lagrelax_knapsack_lbfgs ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxKnapsackLbfgsOutput: Output
{
    LagRelaxKnapsackLbfgsOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number() */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_agents() */
    std::vector<double> multipliers;
};

const LagRelaxKnapsackLbfgsOutput lagrelax_knapsack_lbfgs(
        const Instance& instance,
        const Parameters& parameters = {});

}
