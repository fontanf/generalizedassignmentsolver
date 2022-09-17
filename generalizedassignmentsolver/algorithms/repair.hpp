#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

enum class RepairInitialSolution
{
    CombinatorialRelaxation,
#if COINOR_FOUND
    LinearRelaxationClp,
#endif
#if CPLEX_FOUND
    LinearRelaxationCplex,
#endif
    LagrangianRelaxationKnapsackLbfgs,
};
std::istream& operator>>(std::istream& in, RepairInitialSolution& initial_solution);

struct RepairOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    RepairInitialSolution initial_solution;
    Counter l = -1;
};

struct RepairOutput: Output
{
    RepairOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    RepairOutput& algorithm_end(
            optimizationtools::Info& info);

    Counter iterations = 0;
};

Output repair(
        const Instance& instance,
        std::mt19937_64& generator,
        RepairOptionalParameters parameters = {});

}

