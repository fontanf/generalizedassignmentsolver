#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ColumnGenerationOptionalParameters
{
    optimizationtools::Info info = optimizationtools::Info();

    std::string linear_programming_solver = "CLP";
};

struct ColumnGenerationOutput: Output
{
    ColumnGenerationOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    void print_statistics(
            optimizationtools::Info& info) const override;

    std::vector<double> solution;

    std::vector<std::vector<double>> x;

    Counter number_of_iterations = 0;

    Counter number_of_added_columns = 0;
};

ColumnGenerationOutput columngeneration(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColumnGenerationHeuristicGreedyOutput: Output
{
    ColumnGenerationHeuristicGreedyOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    std::vector<double> solution;

    std::vector<std::vector<double>> x;
};

ColumnGenerationHeuristicGreedyOutput columngenerationheuristic_greedy(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColumnGenerationHeuristicLimitedDiscrepancySearchOutput: Output
{
    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput(
            const Instance& instance,
            optimizationtools::Info& info):
        Output(instance, info) { }

    std::vector<double> solution;

    std::vector<std::vector<double>> x;
};

ColumnGenerationHeuristicLimitedDiscrepancySearchOutput columngenerationheuristic_limiteddiscrepancysearch(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

}

