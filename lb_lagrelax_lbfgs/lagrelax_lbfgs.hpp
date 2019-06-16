#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

/**
 * Relax assignment constraints.
 */

struct LagRelaxAssignmentLbfgsOutput
{
    Cost lb = 0;
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};
LagRelaxAssignmentLbfgsOutput lb_lagrelax_assignment_lbfgs(const Instance& ins, Info info = Info());

/**
 * Relax knapsack constraints.
 */

struct LagRelaxKnapsackLbfgsOutput
{
    Cost lb = 0;
    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};
LagRelaxKnapsackLbfgsOutput lb_lagrelax_knapsack_lbfgs(const Instance& ins, Info info = Info());

}
