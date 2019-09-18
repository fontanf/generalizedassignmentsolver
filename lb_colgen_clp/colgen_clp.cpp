#if COINOR_FOUND

#include "gap/lb_colgen_clp/colgen_clp.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace gap;

void add_column(ColGenClpData& d, ClpSimplex& model, AgentIdx i, std::vector<ItemIdx> col,
        std::vector<double>& ones)
{
    //std::cout << "add_column" << std::endl;
    AgentIdx m = d.ins.agent_number();

    std::vector<int> rows;
    rows.push_back(i);
    Cost c = 0;
    ItemIdx j_prec = 0;
    for (ItemIdx j: col) {
        // Check fixed variables
        AltIdx k = d.ins.alternative_index(j, i);
        if (d.fixed_alt[k] == 0)
            return;
        for (ItemIdx j2 = j_prec + 1; j2 < j; ++j2) {
            AltIdx k2 = d.ins.alternative_index(j2, i);
            if (d.fixed_alt[k2] == 1)
                return;
        }
        j_prec = j;

        rows.push_back(m + j);
        c += d.ins.alternative(k).c;
    }
    std::cout << "add column for agent " << i << " of cost " << c << std::endl;
    model.addColumn(col.size() + 1, rows.data(), ones.data(), 0.0, 1, c);
}

void gap::lb_colgen_clp(ColGenClpData d)
{
    VER(d.info, "*** colgen_clp ***" << std::endl);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    // Initialize solver
    ClpSimplex model;
    for (AgentIdx i=0; i<m; ++i)
        model.addRow(0, NULL, NULL, 0, 1);
    for (ItemIdx j=0; j<n; ++j)
        model.addRow(0, NULL, NULL, 1, n); // or 1?
    std::vector<double> ones(m + n, 1);

    // Add dummy column to ensure feasible solution
    std::vector<int> rows(m + n);
    std::iota(rows.begin(), rows.begin() + n, m);
    model.addColumn(n, rows.data(), ones.data(), 0.0, 1.0, 10 * d.ins.bound());

    // Add initial columns
    if (d.columns.empty()) {
        d.columns.resize(m);
    } else {
        for (AgentIdx i=0; i<m; ++i)
            for (const std::vector<ItemIdx>& column: d.columns[i])
                add_column(d, model, i, column, ones);
    }

    bool found = true;
    while (found) {
        // Solve LP
        model.primal();
        double* dual_sol = model.dualRowSolution();
        std::cout << model.objectiveValue() << std::endl;

        // Find and add new columns
        found = false;
        Weight mult = 10000;
        std::vector<ItemIdx> indices(n);
        for (AgentIdx i=0; i<m; ++i) {
            //std::cout << "knapsack " << i << std::endl;
            knapsack::Instance ins_kp;
            knapsack::Weight capacity_kp = d.ins.capacity(i);
            knapsack::Profit profit_kp = 0;
            ItemIdx j_kp = 0;
            for (ItemIdx j=0; j<n; ++j) {
                AltIdx k = d.ins.alternative_index(j, i);
                if (d.fixed_alt[k] == 0)
                    continue;
                const Alternative& a = d.ins.alternative(k);
                knapsack::Profit p = std::ceil(mult * dual_sol[m + j] - mult * a.c);
                if (d.fixed_alt[k] == 1) {
                    capacity_kp -= a.w;
                    profit_kp += p;
                    continue;
                }
                if (p <= 0)
                    continue;
                ins_kp.add_item(a.w, p);
                indices[j_kp] = j;
                j_kp++;
            }
            ins_kp.set_capacity(capacity_kp);
            knapsack::Solution sol = knapsack::sopt_minknap(ins_kp, knapsack::MinknapParams::combo());
            profit_kp += sol.profit();
            if (profit_kp <= 0)
                continue;
            std::cout << "profit " << profit_kp << std::endl;

            found = true;
            d.columns[i].push_back({});
            for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
                if (sol.contains_idx(j_kp)) {
                    ItemIdx j = indices[j_kp];
                    d.columns[i].back().push_back(j);
                }
            }
            add_column(d, model, i, d.columns[i].back(), ones);
        }
    }

}

#endif

