#pragma once

#include "gap/lib/instance.hpp"

namespace gap
{

class Generator
{

public:

    ItemIdx n = 100; // 100 1000 10000
    double mx = 0.05; // 0 0.05 0.1 0.2 0.33 0.5
    std::string t = "n";
    Weight r = 1000; // 1000 10000 100000
    double x = 0; // 0 0.2 ... 0.8 1
    Seed s = 0;

    Instance generate();

private:

    std::mt19937_64 g;

};

std::ostream& operator<<(std::ostream& os, const Generator& data);

}

