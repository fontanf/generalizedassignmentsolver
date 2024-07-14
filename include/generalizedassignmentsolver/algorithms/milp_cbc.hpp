#pragma once

#if defined(CBC_FOUND) || defined(CLP_FOUND)

#include "generalizedassignmentsolver/solution.hpp"

#include <OsiCbcSolverInterface.hpp>

namespace generalizedassignmentsolver
{

struct CoinLP
{
    CoinLP(const Instance& instance);

    std::vector<double> column_lower_bounds;
    std::vector<double> column_upper_bounds;
    std::vector<double> objective;

    std::vector<double> row_lower_bounds;
    std::vector<double> row_upper_bounds;
    CoinPackedMatrix matrix;
};

}

#endif

#if CBC_FOUND

#include <CbcModel.hpp>

namespace generalizedassignmentsolver
{

struct MilpCbcParameters: Parameters
{
    /** Maximum number of nodes. */
    Counter maximum_number_of_nodes = -1;

    /** Stop at first improvement. */
    bool stop_at_first_improvement = false;

    /** Initial solution. */
    const Solution* initial_solution = NULL;


    virtual int format_width() const override { return 28; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Maximum number of nodes: " << maximum_number_of_nodes << std::endl
            << std::setw(width) << std::left << "Stop at first improvement: " << maximum_number_of_nodes << std::endl
            << std::setw(width) << std::left << "Has initial solution: " << (initial_solution != nullptr) << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                {"MaximumNumberOfNodes", maximum_number_of_nodes},
                {"StopAtFirstImprovement", stop_at_first_improvement},
                {"HasInitialSolution", (initial_solution != nullptr)},
                });
        return json;
    }
};

struct MilpCbcOutput: Output
{
    MilpCbcOutput(
            const Instance& instance):
        Output(instance) { }


    /** Number of nodes. */
    Counter number_of_nodes = 0;


    virtual void format(std::ostream& os) const override
    {
        Output::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Number of nodes: " << number_of_nodes << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Output::to_json();
        json.merge_patch({
                {"NumberOfNodes", number_of_nodes}});
        return json;
    }
};

const MilpCbcOutput milp_cbc(
        const Instance& instance,
        const MilpCbcParameters& parameters = {});

}

#endif
