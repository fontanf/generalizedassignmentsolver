#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "columngenerationsolver/columngenerationsolver.hpp"

#include "knapsacksolver/algorithms/minknap.hpp"
#include "knapsacksolver/algorithms/bellman.hpp"

namespace generalizedassignmentsolver
{

typedef columngenerationsolver::RowIdx RowIdx;
typedef columngenerationsolver::ColIdx ColIdx;
typedef columngenerationsolver::Value Value;
typedef columngenerationsolver::Column Column;
typedef columngenerationsolver::LinearProgrammingSolver LinearProgrammingSolver;

struct ColumnGenerationOptionalParameters
{
    Info info = Info();

    LinearProgrammingSolver linear_programming_solver = LinearProgrammingSolver::CLP;
};

struct ColumnGenerationOutput: Output
{
    ColumnGenerationOutput(const Instance& instance, Info& info): Output(instance, info) { }
    ColumnGenerationOutput& algorithm_end(Info& info);

    std::vector<double> solution;
    std::vector<std::vector<double>> x;
    Counter iteration_number = 0;
    Counter added_column_number = 0;
};

ColumnGenerationOutput columngeneration(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

/******************************************************************************/

struct ColumnGenerationHeuristicGreedyOutput: Output
{
    ColumnGenerationHeuristicGreedyOutput(const Instance& instance, Info& info): Output(instance, info) { }
    ColumnGenerationHeuristicGreedyOutput& algorithm_end(Info& info);

    std::vector<double> solution;
    std::vector<std::vector<double>> x;
};

ColumnGenerationHeuristicGreedyOutput columngenerationheuristic_greedy(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

/******************************************************************************/

struct ColumnGenerationHeuristicLimitedDiscrepancySearchOutput: Output
{
    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput(const Instance& instance, Info& info): Output(instance, info) { }
    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput& algorithm_end(Info& info);

    std::vector<double> solution;
    std::vector<std::vector<double>> x;
};

ColumnGenerationHeuristicLimitedDiscrepancySearchOutput columngenerationheuristic_limiteddiscrepancysearch(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters = {});

}

