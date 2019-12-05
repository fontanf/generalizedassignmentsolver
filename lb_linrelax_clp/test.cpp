#if COINOR_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/lb_linrelax_clp/linrelax_clp.hpp"

using namespace generalizedassignment;

Output lb_linrelax_clp_test(Instance& ins)
{
    return lb_linrelax_clp(ins);
}

std::vector<Output (*)(Instance&)> f = {
        lb_linrelax_clp_test,
};

TEST(LinRelaxClp, TEST) { test(TEST, f, LB); }

#endif

