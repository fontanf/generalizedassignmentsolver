#if CLP_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"

using namespace generalizedassignmentsolver;

Output linrelax_clp_test(Instance& ins)
{
    return linrelax_clp(ins);
}

std::vector<Output (*)(Instance&)> linrelax_clp_tests = {
        linrelax_clp_test,
};

TEST(LinRelaxClp, TEST) { test(TEST, linrelax_clp_tests, LB); }

#endif

