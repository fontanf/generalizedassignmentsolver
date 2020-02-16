#pragma once

#if COINOR_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

/************************* lagrelax_assignment_bundle *************************/

struct LagRelaxAssignmentBundleOutput: Output
{
    LagRelaxAssignmentBundleOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxAssignmentBundleOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};

LagRelaxAssignmentBundleOutput lagrelax_assignment_bundle(const Instance& ins, Info info = Info());

/************************** lagrelax_knapsack_bundle **************************/

struct LagRelaxKnapsackBundleOutput: Output
{
    LagRelaxKnapsackBundleOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxKnapsackBundleOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};

LagRelaxKnapsackBundleOutput lagrelax_knapsack_bundle(const Instance& ins, Info info = Info());

}

#endif

