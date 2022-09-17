#pragma once

#if COINOR_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LinRelaxClpOutput: Output
{
    LinRelaxClpOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    LinRelaxClpOutput& algorithm_end(
            optimizationtools::Info& info);

    std::vector<std::vector<double>> x;
};

LinRelaxClpOutput linrelax_clp(
        const Instance& instance,
        optimizationtools::Info info = optimizationtools::Info());

}

#endif

