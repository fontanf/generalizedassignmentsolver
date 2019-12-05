#if GUROBI_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/lb_linrelax_gurobi/linrelax_gurobi.hpp"

using namespace generalizedassignment;

/*
Output lb_linrelax_gurobi_test(Instance& ins)
{
    return lb_linrelax_gurobi(ins);
}

std::vector<Output (*)(Instance&)> f = {
        lb_linrelax_gurobi_test,
};

TEST(LinRelaxGurobi, TEST) { test(TEST, f, LB); }
*/

#endif

