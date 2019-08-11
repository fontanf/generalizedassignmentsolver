#if COINOR_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

using namespace gap;

Cost sopt_branchandcut_cbc_test(Instance& ins)
{
    Info info = Info()
        .set_verbose(true)
        ;
    Solution sol(ins);
    Cost lb = 0;
    BranchAndCutCbcData d {.ins = ins, .sol = sol, .lb = lb, .stop_at_first_improvment = false, .info = info};
    return sopt_branchandcut_cbc(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        sopt_branchandcut_cbc_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f); }

#endif

