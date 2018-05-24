#include "gap/lib/generator.hpp"

#include "gap/lib/instance.hpp"

#include <random>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace gap;

Instance generate_c(ItemIdx n, AgentIdx m, int obj, int seed)
{
    Instance ins(n, m, obj);
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution_c(5,25);
    std::uniform_int_distribution<int> distribution_p(10,50);
    std::vector<Weight> wsum(m, 0);
    for (ItemIdx j=0; j<n; ++j) {
        ins.add_item();
        for (AgentIdx i=0; i<m; ++i) {
            Weight w = distribution_c(generator);
            Profit p = distribution_p(generator);
            ins.set_alternative(j, i, w, p);
            wsum[i] += w;
        }
    }
    for (AgentIdx i=0; i<m; ++i)
        ins.set_capacity(i, (8*wsum[i]) / (10*m));
    return ins;
}

Instance generate_d(ItemIdx n, AgentIdx m, int obj, int seed)
{
    Instance ins(n, m, obj);
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution_c(1,100);
    std::uniform_int_distribution<int> distribution_e(-10,+10);
    std::vector<Weight> wsum(m, 0);
    for (ItemIdx j=0; j<n; ++j) {
        ins.add_item();
        for (AgentIdx i=0; i<m; ++i) {
            Weight w = distribution_c(generator);
            Weight e = distribution_e(generator);
            Profit p;
            if (obj == 1) {
                p = std::max((Profit)1, 11 + w + e);
            } else {
                p = std::max((Profit)1, 111 - w + e);
            }
            ins.set_alternative(j, i, w, p);
            wsum[i] += w;
        }
    }
    for (AgentIdx i=0; i<m; ++i)
        ins.set_capacity(i, (8*wsum[i]) / (10*m));
    return ins;
}

Instance gap::generate(std::string type, AgentIdx m, ItemIdx n, int obj, int seed)
{
    if (type == "c") {
        return generate_c(n, m, obj, seed);
    } else if (type == "d") {
        return generate_d(n, m, obj, seed);
    } else {
        assert(false);
        return generate_c(n, m, obj, seed);
    }
}

