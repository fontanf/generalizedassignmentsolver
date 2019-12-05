#pragma once

#if GUROBI_FOUND

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

struct LinRelaxGurobiOutput: Output
{
    LinRelaxGurobiOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LinRelaxGurobiOutput& algorithm_end(Info& info);

    std::vector<double> x;
};

LinRelaxGurobiOutput lb_linrelax_gurobi(const Instance& ins, Info info = Info());

}

#endif

