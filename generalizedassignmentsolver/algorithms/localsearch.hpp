#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"

namespace generalizedassignmentsolver
{

/******************************** Local search ********************************/

struct LocalSearchOptionalParameters
{
    Info info = Info();
};

struct LocalSearchOutput: Output
{
    LocalSearchOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LocalSearchOutput& algorithm_end(Info& info);

    Counter it = 0;
};

LocalSearchOutput localsearch(const Instance& ins, std::mt19937_64& gen, LocalSearchOptionalParameters p = {});

/******************************** Tabu search *********************************/

struct TabuSearchOptionalParameters
{
    Info info = Info();

    Counter l = 10000;
};

struct TabuSearchOutput: Output
{
    TabuSearchOutput(const Instance& ins, Info& info): Output(ins, info) { }
    TabuSearchOutput& algorithm_end(Info& info);

    Counter it = 0;
    Counter improving_move_number = 0;
    Counter degrading_move_number = 0;
};

TabuSearchOutput tabusearch(const Instance& ins, std::mt19937_64& gen, TabuSearchOptionalParameters p = {});

/**************************** Simulated annealing *****************************/

struct SimulatedAnnealingOptionalParameters
{
    Info info = Info();

    double beta = 0.999;
    double l = 1000000;
};

struct SimulatedAnnealingOutput: Output
{
    SimulatedAnnealingOutput(const Instance& ins, Info& info): Output(ins, info) { }
    SimulatedAnnealingOutput& algorithm_end(Info& info);

    Counter it = 0;
};

SimulatedAnnealingOutput simulatedannealing(const Instance& ins, std::mt19937_64& gen, SimulatedAnnealingOptionalParameters p = {});

}

