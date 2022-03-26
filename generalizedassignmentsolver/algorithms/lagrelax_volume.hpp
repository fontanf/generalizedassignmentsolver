#pragma once

#if COINOR_FOUND

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////// lagrelax_assignment_volume //////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxAssignmentVolumeOutput: Output
{
    LagRelaxAssignmentVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxAssignmentVolumeOutput& algorithm_end(Info& info);

    std::vector<std::vector<double>> x;
    std::vector<double> multipliers; // vector of size ins.number_of_items()
};

LagRelaxAssignmentVolumeOutput lagrelax_assignment_volume(const Instance& ins, Info info = Info());

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_knapsack_volume ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct LagRelaxKnapsackVolumeOutput: Output
{
    LagRelaxKnapsackVolumeOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LagRelaxKnapsackVolumeOutput& algorithm_end(Info& info);

    std::vector<std::vector<double>> x;
    std::vector<double> multipliers; // vector of size ins.number_of_agents()
};

LagRelaxKnapsackVolumeOutput lagrelax_knapsack_volume(const Instance& ins, Info info = Info());

}

#endif

