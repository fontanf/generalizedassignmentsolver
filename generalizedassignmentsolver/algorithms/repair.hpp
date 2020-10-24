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
    LagrangianRelaxationKnapsackLbfgs,
};
std::istream& operator>>(std::istream& in, RepairInitialSolution& initial_solution);

struct RepairOptionalParameters
{
    Info info = Info();

    RepairInitialSolution initial_solution;
    Counter l = -1;
};

struct RepairOutput: Output
{
    RepairOutput(const Instance& instance, Info& info): Output(instance, info) { }
    RepairOutput& algorithm_end(Info& info);

    Counter iterations = 0;
};

Output repair(
        const Instance& instance,
        std::mt19937_64& generator,
        RepairOptionalParameters parameters = {});

}

