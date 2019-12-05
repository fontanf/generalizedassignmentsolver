#pragma once

#include "generalizedassignment/lib/solution.hpp"

#include "generalizedassignment/lb_linrelax_clp/linrelax_clp.hpp"

namespace generalizedassignment
{

/******************************** Local search ********************************/

struct LSShiftSwapOptionalParameters
{
    Info info = Info();
};

struct LSShiftSwapOutput: Output
{
    LSShiftSwapOutput(const Instance& ins, Info& info): Output(ins, info) { }
    LSShiftSwapOutput& algorithm_end(Info& info);

    Cpt it = 0;
};

LSShiftSwapOutput sol_ls_shiftswap(const Instance& ins, std::mt19937_64& gen, LSShiftSwapOptionalParameters p = {});

/******************************** Tabu search *********************************/

struct TSShiftSwapOptionalParameters
{
    Info info = Info();

    Cpt l = 10000;

    TSShiftSwapOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("l")) != args.end()) l = std::stod(it->second);
        return *this;
    }
};

struct TSShiftSwapOutput: Output
{
    TSShiftSwapOutput(const Instance& ins, Info& info): Output(ins, info) { }
    TSShiftSwapOutput& algorithm_end(Info& info);

    Cpt it = 0;
    Cpt improving_move_number = 0;
    Cpt degrading_move_number = 0;
};

TSShiftSwapOutput sol_ts_shiftswap(const Instance& ins, std::mt19937_64& gen, TSShiftSwapOptionalParameters p = {});

/**************************** Simulated annealing *****************************/

struct SAShiftSwapOptionalParameters
{
    Info info = Info();

    double beta = 0.999;
    double l = 1000000;

    SAShiftSwapOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("beta")) != args.end()) beta = std::stod(it->second);
        if ((it = args.find("l")) != args.end()) l = std::stol(it->second);
        return *this;
    }
};

struct SAShiftSwapOutput: Output
{
    SAShiftSwapOutput(const Instance& ins, Info& info): Output(ins, info) { }
    SAShiftSwapOutput& algorithm_end(Info& info);

    Cpt it = 0;
};

SAShiftSwapOutput sol_sa_shiftswap(const Instance& ins, std::mt19937_64& gen, SAShiftSwapOptionalParameters p = {});

}

