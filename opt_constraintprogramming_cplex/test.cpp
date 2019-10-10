#if CPLEX_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"

using namespace gap;

Output sopt_constraintprogramming_cplex_test(Instance& ins)
{
    ConstraintProgrammingCplexOptionalParameters p;
    return sopt_constraintprogramming_cplex(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        sopt_constraintprogramming_cplex_test,
};

TEST(MILP, TEST) { test(TEST, f, SOPT); }

#endif

