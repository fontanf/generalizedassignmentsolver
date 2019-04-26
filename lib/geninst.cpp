#include "gap/lib/generator.hpp"

using namespace gap;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    std::vector<std::pair<AgentIdx, ItemIdx>> vec = {
        {10, 100},
        {10, 200}, {20, 200},
        {10, 500}, {20, 500}, {50, 500},
        {10, 1000}, {20, 1000}, {50, 1000}, {100, 1000},
    };

    GenerateData data;
    data.r = 100;
    for (std::string t: {"f", "g"}) {
        data.t = t;
        for (auto nm: vec) {
            data.m = nm.first;
            data.n = nm.second;
            for (Seed s=1; s<=5; ++s) {
                data.s = s;
                Instance ins = generate(data);
                ins.write(t + std::to_string(nm.first) + std::to_string(nm.second) + std::to_string(s));
            }
        }
    }

    return 0;
}

