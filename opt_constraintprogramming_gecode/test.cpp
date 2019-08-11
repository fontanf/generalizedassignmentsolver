#if GECODE_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

using namespace gap;

Cost sopt_constraintprogramming_gecode_test(Instance& ins)
{
    Solution sol(ins);
    Cost lb = 0;
    ConstraintProgrammingGecodeData d {.ins = ins, .sol = sol, .lb = lb};
    return sopt_constraintprogramming_gecode(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        sopt_constraintprogramming_gecode_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f); }

#endif

