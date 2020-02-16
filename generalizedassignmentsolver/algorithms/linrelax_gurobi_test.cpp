#if GUROBI_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/linrelax_gurobi.hpp"

using namespace generalizedassignmentsolver;

/*
Output linrelax_gurobi_test(Instance& ins)
{
    return linrelax_gurobi(ins);
}

std::vector<Output (*)(Instance&)> f = {
        linrelax_gurobi_test,
};

TEST(LinRelaxGurobi, TEST) { test(TEST, f, LB); }
*/

#endif

