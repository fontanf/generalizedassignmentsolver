#pragma once

#if GECODE_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ConstraintProgrammingGecodeParameters: Parameters
{
    std::string bound = "none"; // "none", "lagrelax", "colgen"
    std::string search_strategy = "dfs"; // "dfs", "lds"
    std::string branching_strategy = "most_efficient"; // "most_efficient", "most_fractional", "closest_to_one"
    /** if dual = true, branch first on the objective value **/
    bool dual = false;

    ConstraintProgrammingGecodeParameters& primal_solution()
    {
        bound = "lagrelax";
        search_strategy = "lds";
        branching_strategy = "closest_to_one";
        dual = false;
        return *this;
    }

    ConstraintProgrammingGecodeParameters& dual_bound()
    {
        bound = "lagrelax";
        search_strategy = "dfs";
        branching_strategy = "most_fractional";
        dual = true;
        return *this;
    }
};

const Output constraintprogramming_gecode(
        const Instance& instance,
        const ConstraintProgrammingGecodeParameters& parameters = {});

}

#endif

