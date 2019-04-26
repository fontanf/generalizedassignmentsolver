#include "gap/lib/generator.hpp"

#include "gap/lib/instance.hpp"

#include <random>

using namespace gap;

std::ostream& gap::operator<<(std::ostream& os, const GenerateData& data)
{
    os << "n " << data.n << " m " << data.m << " t " << data.t;
    if (data.t == "f")
        os << " r " << data.r;
    if (data.s != 0)
        os << " s " << data.s;
    return os;
}

std::pair<Weight, Value> item(GenerateData& data)
{
    Weight w = -1;
    Value p = -1;
    if (data.t == "a" || data.t == "b" || data.t == "c") {
        std::uniform_int_distribution<Weight> d1(5, 25);
        std::uniform_int_distribution<Weight> d2(10, 50);
        w = d1(data.g);
        p = d2(data.g);
    } else if (data.t == "d") {
        std::uniform_int_distribution<Weight> d1(1, 100);
        std::uniform_int_distribution<Weight> d2(-10, 10);
        w = d1(data.g);
        p = 111 - w + d2(data.g);
    } else if (data.t == "e") { // TODO
    } else if (data.t == "f") {
        std::normal_distribution<double> d(0, data.r / 10);
        w = std::min(data.r, std::max((Weight)1, data.r / 2 + (Weight)d(data.g)));
        p = data.r + 1 - std::min(data.r, std::max((Value)1,  w + (Value)d(data.g)));
    } else {
        exit(1);
    }
    return {w, p};
}

Instance gap::generate(GenerateData data)
{
    Instance ins(data.m, data.n);

    std::vector<Weight> wsum(data.m, 0);
    Weight wmax = 0;
    for (ItemIdx j=0; j<data.n; ++j) {
        ins.add_item();
        for (AgentIdx i=0; i<data.m; ++i) {
            auto wp = item(data);
            ins.set_alternative(j, i, wp.first, wp.second);
            wsum[i] += wp.first;
            wmax = std::max(wmax, wp.first);
        }
    }
    // TODO capacity for types a b c d e
    if (data.t == "f")
        for (AgentIdx i=0; i<data.m; ++i)
            ins.set_capacity(i, std::max(wmax, wsum[i] / data.m));
    return ins;
}

