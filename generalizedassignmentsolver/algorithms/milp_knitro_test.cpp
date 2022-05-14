#if GUROBI_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/milp_gurobi.hpp"

using namespace generalizedassignmentsolver;

// Currently, tests fail. I think that because of Bazel sandboxing, Gurobi is
// unable to find the license file.

/*
Cost milp_gurobi_test(Instance& ins)
{
    Solution sol(ins);
    Cost lb = 0;
    MilpGurobiData d {.ins = ins, .sol = sol, .lb = lb, .info = Info().set_verbose(true)};
    std::cout << "toto" << std::endl;
    return milp_gurobi(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        milp_gurobi_test,
};

TEST(BRANCHANDCUT_GUROBI, TEST) { test(TEST, f); }
*/

#endif

