#if ORTOOLS_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_constraintprogramming_ortools/constraintprogramming_ortools.hpp"

using namespace gap;

Cost sopt_constraintprogramming_gecode_test(Instance& ins)
{
    Solution sol(ins);
    ConstraintProgrammingOrtoolsData d {.ins = ins, .sol = sol};
    return sopt_constraintprogramming_ortools(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        sopt_constraintprogramming_gecode_test,
};

//TEST(BranchAndCutCbc, TEST) { test(TEST, f); }

#endif

