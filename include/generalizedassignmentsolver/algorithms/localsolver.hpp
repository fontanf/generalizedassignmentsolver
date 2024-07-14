#pragma once

#if LOCALSOLVER_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSolverParameters: Parameters
{
};

const Output localsolver(
        const Instance& instance,
        const LocalSolverParameters& parameters = {});

}

#endif

