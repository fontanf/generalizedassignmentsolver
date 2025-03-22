#pragma once

#if CBC_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LinearRelaxationClpOutput: Output
{
    LinearRelaxationClpOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> x;
};

const LinearRelaxationClpOutput linear_relaxation_clp(
        const Instance& instance,
        const Parameters& parameters = {});

}

#endif
