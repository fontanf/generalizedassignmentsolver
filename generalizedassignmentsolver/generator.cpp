#include "generalizedassignmentsolver/generator.hpp"

#include <random>

using namespace generalizedassignmentsolver;

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Generator& data)
{
    os << "n " << data.n
        << " m " << data.m
        << " t " << data.t
        << " r " << data.r
        << " x " << data.x
        << " s " << data.s
        ;
    return os;
}

double truncated_normal(double mean, double stddev, double min, double max, std::mt19937_64& gen)
{
    std::normal_distribution<double> d(mean, stddev);
    for (;;) {
        double res = d(gen);
        if (min <= res && res <= max)
            return res;
    }
}

Instance Generator::generate()
{
    g.seed(s);

    // Item and machine number
    AgentIdx m_eff = std::round(truncated_normal(m, (double)m / 10, 2, 2 * m - 2, g));
    ItemIdx  n_eff = std::round(truncated_normal(n, (double)n / 10, m_eff, 2 * n - m_eff, g));

    Instance ins(m_eff);

    // Weights and profits
    Weight wsum_min = 0;
    Weight wsum_max = 0;
    for (ItemIdx j=0; j<n_eff; ++j) {
        ins.add_item();
        Weight wj = std::round(truncated_normal(r / 2, (double)r / 10, 1, r - 1, g));
        Weight wj_min = r;
        Weight wj_max = 0;
        std::normal_distribution<double> d_wij(wj, wj / 10);
        for (AgentIdx i=0; i<m_eff; ++i) {
            Weight wij = std::round(truncated_normal(wj,      (double)wj        / 10, 1, r - 1, g));
            Cost   cij = std::round(truncated_normal(r - wij, (double)(r - wij) / 10, 1, r - 1, g));
            ins.set_alternative(j, i, wij, cij);
            if (wj_max < wij)
                wj_max = wij;
            if (wj_min > wij)
                wj_min = wij;
        }
        wsum_min += wj_min;
        wsum_max += wj_max;
    }

    // Capacities
    double t = (double)(wsum_min) * (1 - x) + (double)wsum_max * x;
    t = t / m_eff;
    std::normal_distribution<double> d_ti(t, t / 10);
    for (AgentIdx i=0; i<m_eff; ++i) {
        Weight ti = std::ceil(truncated_normal(t, (double)t / 10, 1, 2 * t - 1, g));
        ins.set_capacity(i, ti);
    }

    return ins;
}

