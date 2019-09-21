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
    Weight w = 0;
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
        w += d.ins.alternative(k).w;
    }
    //std::cout << "add column for agent " << i << " of cost " << c << " and weight " << w << "/" << d.ins.capacity(i) << std::endl;
    model.addColumn(col.size() + 1, rows.data(), ones.data(), 0.0, 1, c);
}

Cost gap::lb_colgen_clp(ColGenClpData d)
{
    VER(d.info, "*** colgen_clp ***" << std::endl);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    // Initialize solver
    ClpSimplex model;
    model.messageHandler()->setLogLevel(0);
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
    Weight mult = 1000000;
    while (found) {
        // Solve LP
        model.primal();
        //std::cout << model.objectiveValue() << std::endl;
        double* dual_sol = model.dualRowSolution();

        // Find and add new columns
        found = false;
        std::vector<ItemIdx> indices(n);
        for (AgentIdx i=0; i<m; ++i) {
            // uᵢ: dual value associated with agent constraints (uᵢ ≤ 0).
            // vⱼ: dual value associated with item constraints.
            // xᵢⱼᵏ = 1 if yᵢᵏ contains item j
            //      = 0 otherwise
            // Reduced cost rcᵢᵏ = ∑ⱼ xᵢⱼᵏ cᵢⱼ - (uᵢ + ∑ⱼ xᵢⱼᵏ vⱼ)
            //                   = ∑ⱼ xᵢⱼᵏ (cᵢⱼ - vⱼ) - uᵢ
            // The algorithm that solves KP takes positive integers as input.
            // * positive => the profit in KP are vⱼ - cᵢⱼ and its optimum is
            //   > 0.
            // * integers => we round down the profits to get an upper bound of
            //   the minimum reduced cost.
            // We need an upper bound on the minimum reduced cost in order to
            // know when to stop.
            // At the end, we will need a lower bound on the minimum reduced
            // cost in order to compute the bound.
            // Upper bound on the reduced cost rcubᵢᵏ = - opt(KPfloor) + ⌈-uᵢ⌉
            // Lower bound on the reduced cost rclbᵢᵏ = - opt(KPceil)  + ⌊-uᵢ⌋

            knapsack::Instance ins_kp;
            knapsack::Weight capacity_kp = d.ins.capacity(i);
            Cost rc_ub = std::ceil((mult * (- dual_sol[i])));
            ItemIdx j_kp = 0;
            for (ItemIdx j=0; j<n; ++j) {
                AltIdx k = d.ins.alternative_index(j, i);
                if (d.fixed_alt[k] == 0)
                    continue;
                const Alternative& a = d.ins.alternative(k);
                knapsack::Profit p = std::floor(mult * (dual_sol[m + j] - a.c));
                if (d.fixed_alt[k] == 1) {
                    capacity_kp -= a.w;
                    rc_ub -= std::floor(mult * (dual_sol[m + j] - a.c));
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

            rc_ub -= sol.profit();
            //std::cout << "rc_ub " << rc_ub << std::endl;
            if (rc_ub >= 0)
                continue;

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

    // Compute the bound
    // Solve LP
    model.primal();
    double* dual_sol = model.dualRowSolution();

    std::vector<ItemIdx> indices(n);
    Cost lb = std::floor(mult * model.objectiveValue());
    for (AgentIdx i=0; i<m; ++i) {
        knapsack::Instance ins_kp;
        knapsack::Weight capacity_kp = d.ins.capacity(i);
        Cost rc_lb = std::floor((mult * (- dual_sol[i])));
        ItemIdx j_kp = 0;
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = d.ins.alternative_index(j, i);
            if (d.fixed_alt[k] == 0)
                continue;
            const Alternative& a = d.ins.alternative(k);
            knapsack::Profit p = std::ceil(mult * (dual_sol[m + j] - a.c));
            if (d.fixed_alt[k] == 1) {
                capacity_kp -= a.w;
                rc_lb -= std::ceil(mult * (dual_sol[m + j] - a.c));
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

        rc_lb -= sol.profit();
        lb += rc_lb;
        //std::cout << rc_lb << std::endl;
    }

    d.lb = std::ceil((double)lb / mult);
    return algorithm_end(d.ins, d.lb, d.info);
}

#endif

