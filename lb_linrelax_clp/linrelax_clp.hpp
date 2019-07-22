#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct LinRelaxClpOutput
{
    Cost lb;
    std::vector<double> x;
};

LinRelaxClpOutput lb_linrelax_clp(const Instance& ins, Info info = Info());

}

#endif

