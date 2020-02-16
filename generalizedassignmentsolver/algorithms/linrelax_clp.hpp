#pragma once

#if COINOR_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LinRelaxClpOutput: Output
{
    LinRelaxClpOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LinRelaxClpOutput& algorithm_end(Info& info);

    std::vector<double> x;
};

LinRelaxClpOutput linrelax_clp(const Instance& ins, Info info = Info());

}

#endif

