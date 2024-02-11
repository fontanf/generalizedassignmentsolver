#pragma once

#if CPLEX_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ConstraintProgrammingCplexParameters: Parameters
{
};

const Output constraintprogramming_cplex(
        const Instance& instance,
        const ConstraintProgrammingCplexParameters& parameters = {});

}

#endif

