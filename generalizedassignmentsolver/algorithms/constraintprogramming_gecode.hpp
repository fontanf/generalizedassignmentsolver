#pragma once

#if GECODE_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ConstraintProgrammingGecodeOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    std::string bound = "none"; // "none", "lagrelax", "colgen"
    std::string search_strategy = "dfs"; // "dfs", "lds"
    std::string branching_strategy = "most_efficient"; // "most_efficient", "most_fractional", "closest_to_one"
    /** if dual = true, branch first on the objective value **/
    bool dual = false;

    ConstraintProgrammingGecodeOptionalParameters& primal_solution()
    {
        bound = "lagrelax";
        search_strategy = "lds";
        branching_strategy = "closest_to_one";
        dual = false;
        return *this;
    }

    ConstraintProgrammingGecodeOptionalParameters& dual_bound()
    {
        bound = "lagrelax";
        search_strategy = "dfs";
        branching_strategy = "most_fractional";
        dual = true;
        return *this;
    }
};

Output constraintprogramming_gecode(
        const Instance& instance,
        ConstraintProgrammingGecodeOptionalParameters parameters = {});

}

#endif

