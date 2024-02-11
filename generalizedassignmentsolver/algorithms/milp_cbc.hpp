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
    const Solution* initial_solution = NULL;

    bool stop_at_first_improvment = false;
};

const Output milp_cbc(
        const Instance& instance,
        const MilpCbcParameters& parameters = {});

}

#endif
