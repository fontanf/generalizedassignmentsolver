#pragma once

#include "gap/lib/instance.hpp"

namespace gap
{

struct GenerateData
{
    ItemIdx n = 100;
    AgentIdx m = 10;
    std::string t = "a";
    Weight r = 100;
    Seed s = 0;

    std::default_random_engine g;
};

std::ostream& operator<<(std::ostream& os, const GenerateData& data);

Instance generate(GenerateData data);

}

