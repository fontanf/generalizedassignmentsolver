#include "gap/lib/tester.hpp"
#include "gap/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"

using namespace gap;

Cost opt_constraintprogramming_cplex_test(Instance& ins)
{
    Solution sol(ins);
    ConstraintProgrammingCplexData d {.ins = ins, .sol = sol};
    return sopt_constraintprogramming_cplex(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        opt_constraintprogramming_cplex_test,
};

TEST(MILP, TEST) { test(TEST, f); }

