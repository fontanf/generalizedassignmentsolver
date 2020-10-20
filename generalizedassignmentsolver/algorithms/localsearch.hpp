#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"

namespace generalizedassignmentsolver
{

/******************************** Local search ********************************/

struct LocalSearchOptionalParameters
{
    Info info = Info();

    Counter iteration_limit = -1;
    Counter iteration_without_improvment_limit = -1;

    const Solution* initial_solution = NULL;
};

struct LocalSearchOutput: Output
{
    LocalSearchOutput(const Instance& instance, Info& info): Output(instance, info) { }
    LocalSearchOutput& algorithm_end(Info& info);

    Counter iterations = 0;
};

LocalSearchOutput localsearch(
        const Instance& instance,
        std::mt19937_64& generator,
        LocalSearchOptionalParameters parameters = {});

/******************************** Tabu search *********************************/

struct TabuSearchOptionalParameters
{
    Info info = Info();

    Counter l = 10000;

    Counter iteration_limit = -1;
    Counter iteration_without_improvment_limit = -1;

    const Solution* initial_solution = NULL;
};

struct TabuSearchOutput: Output
{
    TabuSearchOutput(const Instance& instance, Info& info): Output(instance, info) { }
    TabuSearchOutput& algorithm_end(Info& info);

    Counter iterations = 0;
    Counter improving_move_number = 0;
    Counter degrading_move_number = 0;
};

TabuSearchOutput tabusearch(
        const Instance& instance,
        std::mt19937_64& generator,
        TabuSearchOptionalParameters parameters = {});

/**************************** Simulated annealing *****************************/

struct SimulatedAnnealingOptionalParameters
{
    Info info = Info();

    double beta = 0.999;
    double l = 1000000;

    Counter iteration_limit = -1;
    Counter iteration_without_improvment_limit = -1;

    const Solution* initial_solution = NULL;
};

struct SimulatedAnnealingOutput: Output
{
    SimulatedAnnealingOutput(const Instance& instance, Info& info): Output(instance, info) { }
    SimulatedAnnealingOutput& algorithm_end(Info& info);

    Counter iterations = 0;
    Counter iteration_without_improvment_limit = -1;
};

SimulatedAnnealingOutput simulatedannealing(
        const Instance& instance,
        std::mt19937_64& generator,
        SimulatedAnnealingOptionalParameters parameters = {});

}

