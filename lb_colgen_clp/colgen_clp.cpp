#if COINOR_FOUND

#include "gap/lb_colgen_clp/colgen_clp.hpp"

#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace gap;

void add_column(const Instance& ins,
        ColGenClpOptionalParameters& p,
        ClpSimplex& model,
        std::vector<std::vector<std::vector<ItemIdx>>>* columns,
        AgentIdx i,
        ColIdx col_idx,
        std::vector<std::pair<AgentIdx, ColIdx>>& col_indices,
        const std::vector<int>& row,
        std::vector<double>& ones)
{
    std::vector<int> rows;
    rows.push_back(i);
    Cost c = 0;
    Weight w = 0;
    Weight t = ins.capacity(i);
    ItemIdx j_prec = 0;
    for (ItemIdx j: (*columns)[i][col_idx]) {
        AltIdx k = ins.alternative_index(j, i);
        // Check fixed variables
        if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 0)
            return;
        for (ItemIdx j2 = j_prec + 1; j2 < j; ++j2) {
            AltIdx k2 = ins.alternative_index(j2, i);
            if (p.fixed_alt != NULL && (*p.fixed_alt)[k2] == 1)
                return;
        }
        j_prec = j;
        if (row[j] == -1) {
            t -= ins.alternative(k).w;
            continue;
        }

        c += ins.alternative(k).c;
        w += ins.alternative(k).w;
        rows.push_back(row[j]);
    }
    //std::cout << "add column for agent " << i
        //<< " of cost " << c
        //<< " and weight " << w << "/" << t
        //<< " rows.size() " << rows.size() << std::endl;
    col_indices.push_back({i, col_idx});
    model.addColumn(rows.size(), rows.data(), ones.data(), 0.0, 1, c);
}

ColGenClpOutput gap::lb_colgen_clp(const Instance& ins, ColGenClpOptionalParameters p)
{
    VER(p.info, "*** colgen_clp ***" << std::endl);
    ColGenClpOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    std::vector<int> row(n, -2);
    int row_idx = m;
    for (ItemIdx j=0; j<n; ++j) {
        for (AgentIdx i=0; i<m; ++i) {
            if (p.fixed_alt != NULL && (*p.fixed_alt)[ins.alternative_index(j, i)] == 1) {
                row[j] = -1;
                break;
            }
        }
        if (row[j] == -2) {
            row[j] = row_idx;
            row_idx++;
        }
    }

    // Initialize solver
    ClpSimplex model;
    model.messageHandler()->setLogLevel(0);
    for (AgentIdx i=0; i<m; ++i)
        model.addRow(0, NULL, NULL, 0, 1);
    for (ItemIdx j=0; j<n; ++j)
        if (row[j] != -1)
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
    std::vector<knapsack::Weight> kp_capacities(m);
    for (AgentIdx i=0; i<m; ++i) {
        kp_capacities[i] = ins.capacity(i);
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins.alternative_index(j, i);
            if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 1)
                kp_capacities[i] -= ins.alternative(k).w;
        }
        if (kp_capacities[i] < 0)
            std::cout << "ERROR i " << i << " c " << kp_capacities[i] << std::endl;
    }

    // Add initial columns
    model.addColumn(model.numberRows() - m, rows.data(), ones.data(), 0.0, 1.0, 10 * ins.bound());
    column_indices.push_back({-1, -1});
    std::vector<std::vector<std::vector<ItemIdx>>>* columns;
    if (p.columns == NULL) {
        output.columns.resize(m);
        columns = &output.columns;
    } else {
        columns = p.columns;
        for (AgentIdx i=0; i<m; ++i)
            for (ColIdx col_idx = 0; col_idx < (ColIdx)(*columns)[i].size(); ++ col_idx)
                add_column(ins, p, model, columns, i, col_idx, column_indices, row, ones);
    }

    bool found = true;
    Weight mult = 100000;
    while (found) {
        // Solve LP
        model.primal();
        VER(p.info, "T " << std::setw(10) << p.info.elapsed_time() << " | C " << std::setw(10) << model.objectiveValue() << std::endl);
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
            ins_kp.set_capacity(kp_capacities[i]);
            Cost rc_ub = std::ceil((mult * (- dual_sol[i])));
            ItemIdx j_kp = 0;
            for (ItemIdx j=0; j<n; ++j) {
                AltIdx k = ins.alternative_index(j, i);
                const Alternative& a = ins.alternative(k);
                knapsack::Profit profit = std::floor(mult * dual_sol[row[j]]) - std::ceil(mult * a.c);
                if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 1) {
                    indices[j] = -2;
                    continue;
                }
                if ((p.fixed_alt != NULL && (*p.fixed_alt)[k] == 0)
                        || profit <= 0
                        || a.w > kp_capacities[i]) {
                    indices[j] = -1;
                    continue;
                }
                ins_kp.add_item(a.w, profit);
                indices[j] = j_kp;
                j_kp++;
            }
            auto output_kp = knapsack::sopt_minknap(ins_kp);
            rc_ub -= output_kp.solution.profit();
            //std::cout << "rc_ub " << rc_ub << " opt(kp) " << sol.profit() << " vi " << (Cost)std::ceil((mult * (- dual_sol[i]))) << std::endl;
            if (rc_ub >= 0)
                continue;

            found = true;
            (*columns)[i].push_back({});
            for (ItemIdx j = 0; j < n; ++j)
                if (indices[j] == -2
                        || (indices[j] >= 0 && output_kp.solution.contains_idx(indices[j])))
                    (*columns)[i].back().push_back(j);
            add_column(ins, p, model, columns, i, (*columns)[i].size() - 1, column_indices, row, ones);
        }
    }

    // Compute the bound
    // Solve LP
    model.primal();
    double* dual_sol = model.dualRowSolution();

    Cost lb = std::floor(mult * model.objectiveValue());
    for (AgentIdx i=0; i<m; ++i) {
        ins_kp.clear();
        ins_kp.set_capacity(kp_capacities[i]);
        Cost rc_lb = std::floor((mult * (- dual_sol[i])));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins.alternative_index(j, i);
            const Alternative& a = ins.alternative(k);
            knapsack::Profit profit = std::ceil(mult * dual_sol[row[j]]) - std::floor(mult * a.c);
            if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 1)
                continue;
            if ((p.fixed_alt != NULL && (*p.fixed_alt)[k] == 0)
                    || profit <= 0
                    || a.w > kp_capacities[i])
                continue;
            ins_kp.add_item(a.w, profit);
        }
        auto output_kp = knapsack::sopt_minknap(ins_kp);
        rc_lb -= output_kp.solution.profit();
        lb += rc_lb;
        //std::cout << rc_lb << std::endl;
    }

    lb = std::ceil((double)lb / mult);
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 1)
            lb += ins.alternative(k).c;
    output.update_lower_bound(lb, std::stringstream(""), p.info);

    // Compute x
    const double *solution = model.getColSolution();
    std::fill(output.x.begin(), output.x.end(), 0);
    for (ColIdx col_idx = 1; col_idx < model.numberColumns(); ++col_idx) {
        AgentIdx i = column_indices[col_idx].first;
        for (ItemIdx j: (*columns)[i][column_indices[col_idx].second])
            output.x[ins.alternative_index(j, i)] += solution[col_idx];
    }

    return output.algorithm_end(p.info);
}

#endif

