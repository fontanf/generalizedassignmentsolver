#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_assignment_lbfgs //////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxAssignmentLbfgsOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    std::vector<int>* initial_multipliers = NULL;
    std::vector<std::vector<int>>* fixed_alt = NULL; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
};

struct LagRelaxAssignmentLbfgsOutput: Output
{
    LagRelaxAssignmentLbfgsOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    LagRelaxAssignmentLbfgsOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x; // vector of size instance.alternative_number()
    std::vector<double> multipliers; // vector of size instance.number_of_items()
};

LagRelaxAssignmentLbfgsOutput lagrelax_assignment_lbfgs(const Instance& instance, LagRelaxAssignmentLbfgsOptionalParameters p = {});

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// lagrelax_knapsack_lbfgs ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxKnapsackLbfgsOutput: Output
{
    LagRelaxKnapsackLbfgsOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    LagRelaxKnapsackLbfgsOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x; // vector of size instance.alternative_number()
    std::vector<double> multipliers; // vector of size instance.number_of_agents()
};

LagRelaxKnapsackLbfgsOutput lagrelax_knapsack_lbfgs(
        const Instance& instance,
        optimizationtools::Info info = optimizationtools::Info());

}

