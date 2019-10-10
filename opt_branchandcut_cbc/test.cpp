#if COINOR_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

using namespace gap;

Output sopt_branchandcut_cbc_test(Instance& ins)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCbcOptionalParameters p;
    p.info = info;
    return sopt_branchandcut_cbc(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        sopt_branchandcut_cbc_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f, SOPT); }

#endif

