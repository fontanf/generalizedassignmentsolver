#if CPLEX_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_cplex.hpp"

using namespace generalizedassignmentsolver;

Output constraintprogramming_cplex_test(Instance& ins)
{
    ConstraintProgrammingCplexOptionalParameters p;
    return constraintprogramming_cplex(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        constraintprogramming_cplex_test,
};

TEST(MILP, TEST) { test(TEST, f, SOPT); }

#endif

