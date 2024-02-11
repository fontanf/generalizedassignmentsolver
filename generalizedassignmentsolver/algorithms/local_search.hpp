#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSearchParameters: Parameters
{
    Counter number_of_threads = 1;

    const Solution* initial_solution = nullptr;
};

const Output local_search(
        const Instance& instance,
        std::mt19937_64& generator,
        const LocalSearchParameters& parameters = {});

}
