#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

/**
 * Relax assignment constraints.
 */

struct LagRelaxAssignmentVolumeOutput: Output
{
    LagRelaxAssignmentVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};
LagRelaxAssignmentVolumeOutput lb_lagrelax_assignment_volume(const Instance& ins, Info info = Info());

/*ù
 * Relax knapsack constraints.
 */

struct LagRelaxKnapsackVolumeOutput: Output
{
    LagRelaxKnapsackVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};
LagRelaxKnapsackVolumeOutput lb_lagrelax_knapsack_volume(const Instance& ins, Info info = Info());

}

#endif

