#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

struct LocalSearchParameters: Parameters
{
    /** Number of threads. */
    Counter number_of_threads = 1;

    /** Maximum number of nodes. */
    Counter maximum_number_of_nodes = -1;

    /** Initial solution. */
    const Solution* initial_solution = nullptr;


    virtual int format_width() const override { return 26; }

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

const Output local_search(
        const Instance& instance,
        std::mt19937_64& generator,
        const LocalSearchParameters& parameters = {});

}
