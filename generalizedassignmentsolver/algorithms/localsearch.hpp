#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSearchOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    Counter number_of_threads = 1;
    const Solution* initial_solution = nullptr;
};

struct LocalSearchOutput: Output
{
    LocalSearchOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    LocalSearchOutput& algorithm_end(
            optimizationtools::Info& info);
};

LocalSearchOutput localsearch(
        const Instance& instance,
        std::mt19937_64& generator,
        LocalSearchOptionalParameters parameters = {});

}

