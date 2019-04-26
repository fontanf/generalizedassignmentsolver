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

Instance gap::generate(GenerateData data)
{
    Instance ins(data.m, data.n);

    std::uniform_int_distribution<Weight> d1(5, 25);
    std::uniform_int_distribution<Weight> d2(10, 50);
    std::uniform_int_distribution<Weight> d3(1, 100);
    std::uniform_int_distribution<Weight> d4(-10, 10);
    std::normal_distribution<double> d_norm(0, data.r / 10);
    Weight w = -1;
    Value v = -1;

    std::vector<Weight> wsum(data.m, 0);
    Weight wmax = 0;
    for (ItemIdx j=0; j<data.n; ++j) {
        ins.add_item();
        Weight wj = std::min(data.r, std::max((Weight)1, data.r / 2 + (Weight)d_norm(data.g)));
        for (AgentIdx i=0; i<data.m; ++i) {
            if (data.t == "a" || data.t == "b" || data.t == "c") {
                w = d1(data.g);
                v = d2(data.g);
            } else if (data.t == "d") {
                w =           d3(data.g);
                v = 111 - w + d4(data.g);
            } else if (data.t == "e") { // TODO
            } else if (data.t == "f") {
                w =              std::min(data.r, std::max((Weight)1, data.r / 2 + (Weight)d_norm(data.g)));
                v = data.r + 1 - std::min(data.r, std::max((Value)1,  w          + (Value)d_norm(data.g)));
            } else if (data.t == "g") {
                w =              std::min(data.r, std::max((Weight)1, wj + (Weight)d_norm(data.g)));
                v = data.r + 1 - std::min(data.r, std::max((Value)1,  w  + (Value)d_norm(data.g)));
            } else {
                exit(1);
            }
            ins.set_alternative(j, i, w, v);
            wsum[i] += w;
            wmax = std::max(wmax, w);
        }
    }
    for (AgentIdx i=0; i<data.m; ++i) {
        // TODO capacity for types a b
        if (data.t == "c" || data.t == "d" || data.t == "e")
            ins.set_capacity(i, std::max(wmax, (Weight)(0.8 * wsum[i] / data.m)));
        if (data.t == "f" || data.t == "g")
            ins.set_capacity(i, std::max(wmax, wsum[i] / data.m));
    }
    return ins;
}

