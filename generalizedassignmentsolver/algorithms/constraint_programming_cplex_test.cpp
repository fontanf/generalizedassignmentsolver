#if CPLEX_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_cplex.hpp"

using namespace generalizedassignmentsolver;

Output constraintprogramming_cplex_test(Instance& instance)
{
    ConstraintProgrammingCplexOptionalParameters parameters;
    return constraintprogramming_cplex(instance, parameters);
}

std::vector<Output (*)(Instance&)> constraintprogramming_cplex_tests = {
        constraintprogramming_cplex_test,
};

TEST(MILP, TEST) { test(TEST, constraintprogramming_cplex_tests, SOPT); }

#endif

