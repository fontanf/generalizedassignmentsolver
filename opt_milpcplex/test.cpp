#include "gap/lib/tester.hpp"
#include "gap/opt_milpcplex/milpcplex.hpp"

using namespace gap;

Cost opt_milpcplex_test(Instance& ins)
{
    Solution sol(ins);
    MilpCplexData d {.ins = ins, .sol = sol};
    return sopt_milpcplex(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        opt_milpcplex_test,
};

TEST(MILP, TEST) { test(TEST, f); }

