#if COINOR_FOUND

#include "gap/lib/tester.hpp"
#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

using namespace gap;

Cost lb_linrelax_clp_test(Instance& ins)
{
    return lb_linrelax_clp(ins).lb;
}

std::vector<Cost (*)(Instance&)> f = {
        lb_linrelax_clp_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f, LB); }

#endif

