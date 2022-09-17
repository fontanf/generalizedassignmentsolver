#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

Output run(
        std::string algorithm,
        const Instance& instance,
        const Solution& initial_solution,
        std::mt19937_64& generator,
        optimizationtools::Info info);

}

