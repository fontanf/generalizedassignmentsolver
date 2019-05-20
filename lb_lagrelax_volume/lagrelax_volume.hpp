#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

/**
 * Relax assignment constraints.
 */

struct LagRelaxAssignmentVolumeOutput
{
    Cost lb;
    std::vector<double> x;
    std::vector<double> multipliers;
};
LagRelaxAssignmentVolumeOutput lb_lagrelax_assignment_volume(const Instance& ins, Info info = Info());

/*ù
 * Relax knapsack constraints.
 */

struct LagRelaxKnapsackVolumeOutput
{
    Cost lb;
    std::vector<double> x;
    std::vector<double> multipliers;
};
LagRelaxKnapsackVolumeOutput lb_lagrelax_knapsack_volume(const Instance& ins, Info info = Info());

}

