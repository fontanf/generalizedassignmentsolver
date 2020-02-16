#pragma once

#if DLIB_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

/************************** lagrelax_assignment_lbfgs *************************/

struct LagRelaxAssignmentLbfgsOptionalParameters
{
    Info info = Info();

    std::vector<int>* initial_multipliers = NULL;
    std::vector<int>* fixed_alt = NULL; // -1: unfixed, 0: fixed to 0, 1: fixed to 1.
};

struct LagRelaxAssignmentLbfgsOutput: Output
{
    LagRelaxAssignmentLbfgsOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxAssignmentLbfgsOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.item_number()
};

LagRelaxAssignmentLbfgsOutput lagrelax_assignment_lbfgs(const Instance& ins, LagRelaxAssignmentLbfgsOptionalParameters p = {});

/*************************** lagrelax_knapsack_lbfgs **************************/

struct LagRelaxKnapsackLbfgsOutput: Output
{
    LagRelaxKnapsackLbfgsOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxKnapsackLbfgsOutput& algorithm_end(Info& info);

    std::vector<double> x; // vector of size ins.alternative_number()
    std::vector<double> multipliers; // vector of size ins.agent_number()
};

LagRelaxKnapsackLbfgsOutput lagrelax_knapsack_lbfgs(const Instance& ins, Info info = Info());

}

#endif

