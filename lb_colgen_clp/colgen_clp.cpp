#if COINOR_FOUND

#include "gap/lb_colgen_clp/colgen_clp.hpp"

#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace gap;

void add_column(ColGenClpData& d, ClpSimplex& model, AgentIdx i, ColIdx col_idx,
        std::vector<std::pair<AgentIdx, ColIdx>>& col_indices, std::vector<double>& ones)
{
    AgentIdx m = d.ins.agent_number();

    std::vector<int> rows;
    rows.push_back(i);
    Cost c = 0;
    Weight w = 0;
    ItemIdx j_prec = 0;
    for (ItemIdx j: d.columns[i][col_idx]) {
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
    col_indices.push_back({i, col_idx});
    model.addColumn(d.columns[i][col_idx].size() + 1, rows.data(), ones.data(), 0.0, 1, c);
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
        model.addRow(0, NULL, NULL, 1, n);
    std::vector<double> ones(m + n, 1);

    // Add dummy column to ensure feasible solution
    std::vector<int> rows(m + n);
    std::iota(rows.begin(), rows.begin() + n, m);
    std::vector<std::pair<AgentIdx, ColIdx>> column_indices;

    // KP utils
    knapsack::Instance ins_kp;
    // indices[j] == -2: item j is not in KP but belong to the column.
    //            == -1: item j is not in KP and does not belong to the column.
    //            >=  0: item j is the item of index indices[j] in KP.
    std::vector<knapsack::ItemIdx> indices(n);

    // Add initial columns
    if (d.columns.empty()) {
        d.columns.resize(m);
        model.addColumn(n, rows.data(), ones.data(), 0.0, 1.0, 10 * d.ins.bound());
        column_indices.push_back({-1, -1});
    } else {
        for (AgentIdx i=0; i<m; ++i)
            for (ColIdx col_idx = 0; col_idx < (ColIdx)d.columns[i].size(); ++ col_idx)
                add_column(d, model, i, col_idx, column_indices, ones);
    }

    bool found = true;
    Weight mult = 100000;
    while (found) {
        // Solve LP
        model.primal();
        VER(d.info, "T " << std::setw(10) << d.info.elapsed_time() << " | C " << std::setw(10) << model.objectiveValue() << std::endl);
        double* dual_sol = model.dualRowSolution();

        // Find and add new columns
        found = false;
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

            ins_kp.clear();
            knapsack::Weight capacity_kp = d.ins.capacity(i);
            Cost rc_ub = std::ceil((mult * (- dual_sol[i])));
            ItemIdx j_kp = 0;
            for (ItemIdx j=0; j<n; ++j) {
                AltIdx k = d.ins.alternative_index(j, i);
                const Alternative& a = d.ins.alternative(k);
                //knapsack::Profit p = std::floor(mult * (dual_sol[m + j] - a.c));
                knapsack::Profit p = std::floor(mult * dual_sol[m + j]) - std::ceil(mult * a.c);
                if (d.fixed_alt[k] == 1) {
                    capacity_kp -= a.w;
                    rc_ub -= p;
                    indices[j] = -2;
                    continue;
                }
                if (d.fixed_alt[k] == 0 || p <= 0) {
                    indices[j] = -1;
                    continue;
                }
                ins_kp.add_item(a.w, p);
                indices[j] = j_kp;
                j_kp++;
            }
            ins_kp.set_capacity(capacity_kp);
            knapsack::Solution sol = knapsack::sopt_minknap(ins_kp, knapsack::MinknapParams::combo());
            rc_ub -= sol.profit();
            //std::cout << "rc_ub " << rc_ub << " opt(kp) " << sol.profit() << " vi " << (Cost)std::ceil((mult * (- dual_sol[i]))) << std::endl;
            if (rc_ub >= 0)
                continue;

            found = true;
            d.columns[i].push_back({});
            for (ItemIdx j = 0; j < n; ++j)
                if (indices[j] == -2 || (indices[j] >= 0 && sol.contains_idx(indices[j])))
                    d.columns[i].back().push_back(j);
            add_column(d, model, i, d.columns[i].size() - 1, column_indices, ones);
        }
    }

    // Compute the bound
    // Solve LP
    model.primal();
    double* dual_sol = model.dualRowSolution();

    Cost lb = std::floor(mult * model.objectiveValue());
    for (AgentIdx i=0; i<m; ++i) {
        ins_kp.clear();
        knapsack::Weight capacity_kp = d.ins.capacity(i);
        Cost rc_lb = std::floor((mult * (- dual_sol[i])));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = d.ins.alternative_index(j, i);
            const Alternative& a = d.ins.alternative(k);
            //knapsack::Profit p = std::ceil(mult * (dual_sol[m + j] - a.c));
            knapsack::Profit p = std::ceil(mult * dual_sol[m + j]) - std::floor(mult * a.c);
            if (d.fixed_alt[k] == 1) {
                capacity_kp -= a.w;
                rc_lb -= p;
                continue;
            }
            if (d.fixed_alt[k] == 0 || p <= 0)
                continue;
            ins_kp.add_item(a.w, p);
        }
        ins_kp.set_capacity(capacity_kp);
        knapsack::Solution sol = knapsack::sopt_minknap(ins_kp, knapsack::MinknapParams::combo());
        rc_lb -= sol.profit();
        lb += rc_lb;
        //std::cout << rc_lb << std::endl;
    }

    d.lb = std::ceil((double)lb / mult);
    const double *solution = model.getColSolution();
    std::fill(d.x.begin(), d.x.end(), 0);
    for (ColIdx col_idx = 1; col_idx < model.numberColumns(); ++col_idx) {
        AgentIdx i = column_indices[col_idx].first;
        for (ItemIdx j: d.columns[i][column_indices[col_idx].second])
            d.x[d.ins.alternative_index(j, i)] += solution[col_idx];
    }
    return algorithm_end(d.ins, d.lb, d.info);
}

#endif

