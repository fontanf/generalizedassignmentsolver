#pragma once

#if COINOR_FOUND

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

/*********************** lb_lagrelax_assignment_volume ************************/

struct LagRelaxAssignmentVolumeOutput: Output
{
    LagRelaxAssignmentVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxAssignmentVolumeOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};

LagRelaxAssignmentVolumeOutput lb_lagrelax_assignment_volume(const Instance& ins, Info info = Info());

/************************ lb_lagrelax_knapsack_volume *************************/

struct LagRelaxKnapsackVolumeOutput: Output
{
    LagRelaxKnapsackVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxKnapsackVolumeOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};

LagRelaxKnapsackVolumeOutput lb_lagrelax_knapsack_volume(const Instance& ins, Info info = Info());

}

#endif

