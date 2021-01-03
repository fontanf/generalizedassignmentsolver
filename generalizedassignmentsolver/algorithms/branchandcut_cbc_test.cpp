#if COINOR_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cbc.hpp"

using namespace generalizedassignmentsolver;

Output branchandcut_cbc_test(Instance& instance)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCbcOptionalParameters parameters;
    parameters.info = info;
    return branchandcut_cbc(instance, parameters);
}

std::vector<Output (*)(Instance&)> branchandcut_cbc_tests = {
        branchandcut_cbc_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, branchandcut_cbc_tests, SOPT); }

#endif

