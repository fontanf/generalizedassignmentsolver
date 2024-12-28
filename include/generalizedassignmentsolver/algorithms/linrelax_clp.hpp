#pragma once

#if CBC_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LinRelaxClpOutput: Output
{
    LinRelaxClpOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;
};

const LinRelaxClpOutput linrelax_clp(
        const Instance& instance,
        const Parameters& parameters = {});

}

#endif
