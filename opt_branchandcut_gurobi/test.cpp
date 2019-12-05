#if GUROBI_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"

using namespace generalizedassignment;

// Currently, tests fail. I think that because of Bazel sandboxing, Gurobi is
// unable to find the license file.

/*
Cost opt_branchandcut_gurobi_test(Instance& ins)
{
    Solution sol(ins);
    Cost lb = 0;
    BranchAndCutGurobiData d {.ins = ins, .sol = sol, .lb = lb, .info = Info().set_verbose(true)};
    std::cout << "toto" << std::endl;
    return sopt_branchandcut_gurobi(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        opt_branchandcut_gurobi_test,
};

TEST(BRANCHANDCUT_GUROBI, TEST) { test(TEST, f); }
*/

#endif

