#pragma once

#include "generalizedassignment/lib/instance.hpp"

namespace generalizedassignment
{

class Generator
{

public:

    ItemIdx n = 100;
    AgentIdx m = 10;
    std::string t = "n";
    Weight r = 1000;
    double x = 0;
    Seed s = 0;

    Instance generate();

private:

    std::mt19937_64 g;

};

std::ostream& operator<<(std::ostream& os, const Generator& data);

}

