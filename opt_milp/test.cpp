#include "gap/lib/tester.hpp"
#include "gap/opt_milp/milp.hpp"

using namespace gap;

Value opt_milp_test(Instance& ins) { return sopt_milp(ins).value(); }

std::vector<Value (*)(Instance&)> f = {
        opt_milp_test,
};

TEST(MILP, TEST) { test(TEST, f); }

