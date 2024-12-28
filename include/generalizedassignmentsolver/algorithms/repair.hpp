#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

enum class RepairInitialSolution
{
    CombinatorialRelaxation,
#if CBC_FOUND
    LinearRelaxationClp,
#endif
#if CPLEX_FOUND
    LinearRelaxationCplex,
#endif
    LagrangianRelaxationKnapsackLbfgs,
};
std::istream& operator>>(std::istream& in, RepairInitialSolution& initial_solution);

struct RepairParameters: Parameters
{
    RepairInitialSolution initial_solution;

    Counter l = -1;
};

struct RepairOutput: Output
{
    RepairOutput(
            const Instance& instance):
        Output(instance) { }


    /** Number of iterations. */
    Counter iterations = 0;
};

RepairOutput repair(
        const Instance& instance,
        std::mt19937_64& generator,
        const RepairParameters& parameters = {});

}

