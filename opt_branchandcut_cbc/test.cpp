#if COINOR_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

using namespace gap;

Cost sopt_branchandcut_cbc_test(Instance& ins)
{
    Solution sol(ins);
    BranchAndCutCbcData d {.ins = ins, .sol = sol};
    return sopt_branchandcut_cbc(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        sopt_branchandcut_cbc_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f); }

#endif

