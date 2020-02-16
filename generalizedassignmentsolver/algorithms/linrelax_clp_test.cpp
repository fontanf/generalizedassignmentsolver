#if COINOR_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"

using namespace generalizedassignmentsolver;

Output linrelax_clp_test(Instance& ins)
{
    return linrelax_clp(ins);
}

std::vector<Output (*)(Instance&)> f = {
        linrelax_clp_test,
};

TEST(LinRelaxClp, TEST) { test(TEST, f, LB); }

#endif

