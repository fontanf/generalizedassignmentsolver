#if CPLEX_FOUND

#include "gap/lb_colgen_cplex/colgen_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

ColGenCplexOutput& ColGenCplexOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "Iterations", it);
    VER(info, "Iterations: " << it << std::endl);
    PUT(info, "Algorithm", "AddedColumns", added_column_number);
    VER(info, "Added columns: " << added_column_number << std::endl);
    return *this;
}

void add_column(const Instance& ins,
        ColGenCplexOptionalParameters& p,
        IloModel& model,
        IloObjective& obj,
        IloRangeArray& range,
        std::vector<std::vector<std::vector<ItemIdx>>>* columns,
        AgentIdx i,
        ColIdx col_idx,
        std::vector<std::pair<AgentIdx, ColIdx>>& col_indices,
        const std::vector<int>& row)
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
    col_indices.push_back({i, col_idx});

    IloNumColumn col = obj(c);
    for (int i: rows)
        col += range[i](1);
    model.add(IloNumVar(col, 0, 1, ILOFLOAT));
}

ColGenCplexOutput gap::lb_colgen_cplex(const Instance& ins, ColGenCplexOptionalParameters p)
{
    VER(p.info, "*** colgen_cplex ***" << std::endl);
    ColGenCplexOutput output(ins, p.info);

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
    IloEnv env;
    IloModel model(env);
    IloObjective obj(env);
    obj.setSense(IloObjective::Minimize);
    model.add(obj);
    IloRangeArray range(env);
    for (AgentIdx i = 0; i < m; ++i)
        range.add(IloRange(env, -IloInfinity, 1));
    for (ItemIdx j = 0; j < n; ++j)
        range.add(IloRange(env, 1, IloInfinity));
    model.add(range);
    IloCplex cplex(model);
    cplex.setOut(env.getNullStream()); // Remove standard output

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
    IloNumColumn col = obj(10 * ins.bound());
    for (int i = m; i < row_idx; ++i)
        col += range[i](1);
    model.add(IloNumVar(col, 0, 1, ILOFLOAT));

    column_indices.push_back({-1, -1});
    std::vector<std::vector<std::vector<ItemIdx>>>* columns;
    if (p.columns == NULL) {
        output.columns.resize(m);
        columns = &output.columns;
    } else {
        columns = p.columns;
        for (AgentIdx i=0; i<m; ++i)
            for (ColIdx col_idx = 0; col_idx < (ColIdx)(*columns)[i].size(); ++ col_idx)
                add_column(ins, p, model, obj, range, columns, i, col_idx, column_indices, row);
    }

    bool found = true;
    Weight mult = 100000;
    IloNumArray dual_sol(env, row_idx);
    while (found) {
        output.it++;

        // Solve LP
        cplex.solve();
        VER(p.info,
                "It " << std::setw(8) << output.it
                << " | T " << std::setw(10) << p.info.elapsed_time()
                << " | C " << std::setw(10) << cplex.getObjValue()
                << " | COL " << std::setw(10) << output.added_column_number
                << std::endl);
        cplex.getDuals(dual_sol, range);

        // Find and add new columns
        found = false;
        for (AgentIdx i=0; i<m; ++i) {
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
            add_column(ins, p, model, obj, range, columns, i, (*columns)[i].size() - 1, column_indices, row);
            output.added_column_number++;
        }
    }

    // Compute the bound
    // Solve LP
    cplex.solve();
    cplex.getDuals(dual_sol, range);

    Cost lb = std::floor(mult * cplex.getObjValue());
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
    //const double *solution = model.getColSolution();
    //output.x.resize(ins.alternative_number(), 0);
    //for (ColIdx col_idx = 1; col_idx < model.numberColumns(); ++col_idx) {
        //AgentIdx i = column_indices[col_idx].first;
        //for (ItemIdx j: (*columns)[i][column_indices[col_idx].second])
            //output.x[ins.alternative_index(j, i)] += solution[col_idx];
    //}

    return output.algorithm_end(p.info);
}

#endif

