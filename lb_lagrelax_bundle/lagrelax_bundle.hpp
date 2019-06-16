#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

/**
 * Relax assignment constraints.
 */

struct LagRelaxAssignmentBundleOutput
{
    Cost lb = 0;
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};
LagRelaxAssignmentBundleOutput lb_lagrelax_assignment_bundle(const Instance& ins, Info info = Info());

/**
 * Relax knapsack constraints.
 */

struct LagRelaxKnapsackBundleOutput
{
    Cost lb = 0;
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};
LagRelaxKnapsackBundleOutput lb_lagrelax_knapsack_bundle(const Instance& ins, Info info = Info());

}

