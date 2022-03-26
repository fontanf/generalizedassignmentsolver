#pragma once

#if COINOR_FOUND

#include "generalizedassignmentsolver/solution.hpp"

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CglKnapsackCover.hpp>
#include <coin/CglClique.hpp>

namespace generalizedassignmentsolver
{

struct CoinLP
{
    CoinLP(const Instance& instance);

    std::vector<double> col_lower;
    std::vector<double> col_upper;
    std::vector<double> objective;

    std::vector<double> row_lower;
    std::vector<double> row_upper;
    CoinPackedMatrix matrix;
};

struct MilpCbcOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
    bool stop_at_first_improvment = false;
};

struct MilpCbcOutput: Output
{
    MilpCbcOutput(
            const Instance& instance,
            Info& info):
        Output(instance, info) { }

    MilpCbcOutput& algorithm_end(Info& info);
};

MilpCbcOutput milp_cbc(
        const Instance& instance,
        MilpCbcOptionalParameters parameters = {});

}

#endif

