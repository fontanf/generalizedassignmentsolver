#include "gap/lib/generator.hpp"

#include <random>

using namespace gap;

std::ostream& gap::operator<<(std::ostream& os, const Generator& data)
{
    os << "n " << data.n
        << " mx " << data.mx
        << " t " << data.t
        << " r " << data.r
        << " x " << data.x
        << " s " << data.s
        ;
    return os;
}

Instance Generator::generate()
{
    g.seed(s);
    AgentIdx m = n * mx;
    Instance ins(m);
    std::normal_distribution<double> d_wj(r / 2, r / 20);
    Weight wsum_min = 0;
    Weight wsum_max = 0;
    for (ItemIdx j=0; j<n; ++j) {
        ins.add_item();
        Weight wj = d_wj(g);
        Weight wj_min = r;
        Weight wj_max = 0;
        std::normal_distribution<double> d_wij(wj, wj / 10);
        for (AgentIdx i=0; i<m; ++i) {
            Weight wij;
            do {
                wij = d_wij(g);
            } while (wij <= 0 || wij > r);
            std::normal_distribution<double> d_cij(r - wij, (r - wij) / 10);
            Cost cij;
            do {
                cij = d_cij(g);
            } while (cij <= 0 || cij > r);
            ins.set_alternative(j, i, wij, cij);
            if (wj_max < wij)
                wj_max = wij;
            if (wj_min > wij)
                wj_min = wij;
        }
        wsum_min += wj_min;
        wsum_max += wj_max;
    }
    double c = (double)(wsum_min) * (1 - x) + (double)wsum_max * x;
    c = c / m;
    for (AgentIdx i=0; i<m; ++i)
        ins.set_capacity(i, std::ceil(c));
    return ins;
}

