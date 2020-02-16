#pragma once

#if GUROBI_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LinRelaxGurobiOutput: Output
{
    LinRelaxGurobiOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LinRelaxGurobiOutput& algorithm_end(Info& info);

    std::vector<double> x;
};

LinRelaxGurobiOutput linrelax_gurobi(const Instance& ins, Info info = Info());

}

#endif

