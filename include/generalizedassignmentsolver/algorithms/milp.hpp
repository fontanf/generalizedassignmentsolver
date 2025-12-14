#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include "mathoptsolverscmake/common.hpp"

namespace generalizedassignmentsolver
{

struct MilpParameters: Parameters
{
    /** Solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Highs;

    /** Maximum number of nodes. */
    Counter maximum_number_of_nodes = -1;

    /** Initial solution. */
    const Solution* initial_solution = NULL;


    virtual int format_width() const override { return 28; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Maximum number of nodes: " << maximum_number_of_nodes << std::endl
            << std::setw(width) << std::left << "Has initial solution: " << (initial_solution != nullptr) << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                {"MaximumNumberOfNodes", maximum_number_of_nodes},
                {"HasInitialSolution", (initial_solution != nullptr)},
                });
        return json;
    }
};

struct MilpOutput: Output
{
    MilpOutput(
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

MilpOutput milp(
        const Instance& instance,
        const Solution* initial_solution = nullptr,
        const MilpParameters& parameters = {});

}
