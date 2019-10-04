#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

struct LinRelaxClpOutput: Output
{
    LinRelaxClpOutput(const Instance& ins, Info& info): Output(ins, info) { }
    std::vector<double> x;
};

LinRelaxClpOutput lb_linrelax_clp(const Instance& ins, Info info = Info());

}

#endif

