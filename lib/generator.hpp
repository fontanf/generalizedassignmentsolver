#pragma once

#include "gap/lib/instance.hpp"

namespace gap
{

Instance generate(std::string type, AgentIdx m, ItemIdx n, int obj, int seed);

struct GenParams
{
    AgentIdx m;
    ItemIdx n;
    Value r;
    int bp;
    int bw;
    int ep;
    int ew;
    int h;
    int hmax;
    int seed;
};

std::ostream& operator<<(std::ostream &os, const GenParams& p);

Instance generate(const GenParams& p);

class GenParamsFactory
{

public:

    const GenParams P0W0E0{5, 25, 100, 0,  0, 0, 0, 75, 100, 1};
    const GenParams P0W1E0{5, 25, 100, 0,  1, 0, 0, 75, 100, 1};
    const GenParams P0W2E0{5, 25, 100, 0, -1, 0, 0, 75, 100, 1};
    const GenParams P1W0E0{5, 25, 100, 1,  0, 0, 0, 75, 100, 1};
    const GenParams P1W1E0{5, 25, 100, 1,  1, 0, 0, 75, 100, 1};
    const GenParams P1W2E0{5, 25, 100, 1, -1, 0, 0, 75, 100, 1};
    const GenParams P0W0E1{5, 25, 100, 0,  0, 1, 1, 75, 100, 1};
    const GenParams P0W1E1{5, 25, 100, 0,  1, 1, 1, 75, 100, 1};
    const GenParams P0W2E1{5, 25, 100, 0, -1, 1, 1, 75, 100, 1};
    const GenParams P1W0E1{5, 25, 100, 1,  0, 1, 1, 75, 100, 1};
    const GenParams P1W1E1{5, 25, 100, 1,  1, 1, 1, 75, 100, 1};
    const GenParams P1W2E1{5, 25, 100, 1, -1, 1, 1, 75, 100, 1};

private:

};

}

