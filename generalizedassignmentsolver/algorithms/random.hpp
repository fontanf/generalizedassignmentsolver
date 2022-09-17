#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

Solution random_infeasible(
        const Instance& instance,
        std::mt19937_64& generator);

Output random(
        const Instance& instance,
        std::mt19937_64& generator,
        optimizationtools::Info info = optimizationtools::Info());

}

