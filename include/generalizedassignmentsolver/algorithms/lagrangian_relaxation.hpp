#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "mathoptsolverscmake/common.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
//////////////////////// lagrangian_relaxation_assignment //////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationAssignmentParameters: Parameters
{
    /** Solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Dlib;
};

struct LagrangianRelaxationAssignmentOutput: Output
{
    LagrangianRelaxationAssignmentOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationAssignmentOutput lagrangian_relaxation_assignment(
        const Instance& instance,
        std::vector<double>* initial_multipliers = NULL,
        std::vector<std::vector<int>>* fixed_alt = NULL,
        const LagrangianRelaxationAssignmentParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
////////////////////////// lagrangian_relaxation_knapsack //////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagrangianRelaxationKnapsackParameters: Parameters
{
    /** Solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Dlib;
};

struct LagrangianRelaxationKnapsackOutput: Output
{
    LagrangianRelaxationKnapsackOutput(
            const Instance& instance):
        Output(instance) { }

    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

const LagrangianRelaxationKnapsackOutput lagrangian_relaxation_knapsack(
        const Instance& instance,
        std::vector<double>* initial_multipliers = NULL,
        std::vector<std::vector<int>>* fixed_alt = NULL,
        const LagrangianRelaxationKnapsackParameters& parameters = {});

}
