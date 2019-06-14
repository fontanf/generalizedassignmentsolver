#include "gap/lib/tester.hpp"
#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

using namespace gap;

Cost sopt_constraintprogramming_gecode_test(Instance& ins)
{
    Solution sol(ins);
    ConstraintProgrammingGecodeData d {.ins = ins, .sol = sol};
    return sopt_constraintprogramming_gecode(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        sopt_constraintprogramming_gecode_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f); }

