#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct ColumnGenerationParameters: Parameters
{
    std::string linear_programming_solver = "CLP";


    virtual int format_width() const override { return 37; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Linear programming solver: " << linear_programming_solver << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                {"LinearProgrammingSolver", linear_programming_solver},
                });
        return json;
    }
};

struct ColumnGenerationOutput: Output
{
    ColumnGenerationOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> relaxation_solution;

    Counter number_of_iterations = 0;

    Counter number_of_added_columns = 0;


    virtual void format(std::ostream& os) const override
    {
        Output::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Number of iterations: " << number_of_iterations << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Output::to_json();
        json.merge_patch({
                {"NumberOfIterations", number_of_iterations}});
        return json;
    }
};

const ColumnGenerationOutput column_generation(
        const Instance& instance,
        const ColumnGenerationParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColumnGenerationHeuristicGreedyOutput: Output
{
    ColumnGenerationHeuristicGreedyOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> relaxation_solution;
};

const ColumnGenerationHeuristicGreedyOutput column_generation_heuristic_greedy(
        const Instance& instance,
        const ColumnGenerationParameters& parameters = {});

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColumnGenerationHeuristicLimitedDiscrepancySearchOutput: Output
{
    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput(
            const Instance& instance):
        Output(instance) { }


    std::vector<std::vector<double>> relaxation_solution;
};

const ColumnGenerationHeuristicLimitedDiscrepancySearchOutput column_generation_heuristic_limited_discrepancy_search(
        const Instance& instance,
        const ColumnGenerationParameters& parameters = {});

}
