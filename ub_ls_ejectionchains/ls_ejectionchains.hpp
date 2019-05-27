#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

namespace gap
{

struct LSFirstECData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    PCost alpha = 100;

    LSFirstECData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);
        return *this;
    }
};
void sol_lsfirst_ejectionchain(LSFirstECData d, Solution& sol_best);

}

