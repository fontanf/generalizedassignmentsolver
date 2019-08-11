#pragma once

#if GUROBI_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct LinRelaxGurobiOutput
{
    Cost lb;
    std::vector<double> x;
};

LinRelaxGurobiOutput lb_linrelax_gurobi(const Instance& ins, Info info = Info());

}

#endif

