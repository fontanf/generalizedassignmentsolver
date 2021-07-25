#if COINOR_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"

using namespace generalizedassignmentsolver;

Output milp_cbc_test(Instance& instance)
{
    Info info = Info()
        .set_verbose(true)
        ;
    MilpCbcOptionalParameters parameters;
    parameters.info = info;
    return milp_cbc(instance, parameters);
}

std::vector<Output (*)(Instance&)> milp_cbc_tests = {
        milp_cbc_test,
};

TEST(MilpCbc, TEST) { test(TEST, milp_cbc_tests, SOPT); }

#endif

