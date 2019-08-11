#if GUROBI_FOUND

#include "gap/lib/tester.hpp"
#include "gap/lb_linrelax_gurobi/linrelax_gurobi.hpp"

using namespace gap;

Cost lb_linrelax_gurobi_test(Instance& ins)
{
    return lb_linrelax_gurobi(ins).lb;
}

std::vector<Cost (*)(Instance&)> f = {
        lb_linrelax_gurobi_test,
};

TEST(LinRelaxGurobi, TEST) { test(TEST, f, LB); }

#endif

