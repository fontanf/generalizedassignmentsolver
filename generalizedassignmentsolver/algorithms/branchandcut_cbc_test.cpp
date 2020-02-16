#if COINOR_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cbc.hpp"

using namespace generalizedassignmentsolver;

Output branchandcut_cbc_test(Instance& ins)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCbcOptionalParameters p;
    p.info = info;
    return branchandcut_cbc(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        branchandcut_cbc_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f, SOPT); }

#endif

