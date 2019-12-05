#if COINOR_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/opt_branchandcut_cbc/branchandcut_cbc.hpp"

using namespace generalizedassignment;

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

