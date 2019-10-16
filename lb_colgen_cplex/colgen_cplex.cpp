
#include "gap/lb_colgen_cplex/colgen_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace gap;

class ColGenSolver
{

public:

    virtual void add_column(std::vector<int> rows, Cost c) = 0;
    virtual void solve() = 0;
    virtual double objective() = 0;
    virtual std::vector<double>& dual_solution() = 0;
    virtual std::vector<double> solution() = 0;
};

#if CPLEX_FOUND

ILOSTLBEGIN

class ColGenSolverCplex: public ColGenSolver
{

public:

    ColGenSolverCplex(AgentIdx agent_constraint_number, ItemIdx item_constraint_number):
        m_(agent_constraint_number),
        n_(item_constraint_number),
        model_(env_),
        obj_(env_),
        range_(env_),
        cplex_(model_),
        dual_sol_(env_, m_ + n_),
        dual_sol_vd_(m_ + n_)
    {
        obj_.setSense(IloObjective::Minimize);
        model_.add(obj_);
        for (AgentIdx i = 0; i < m_; ++i)
            range_.add(IloRange(env_, -IloInfinity, 1));
        for (ItemIdx j = 0; j < n_; ++j)
            range_.add(IloRange(env_, 1, IloInfinity));
        model_.add(range_);
        cplex_.setOut(env_.getNullStream()); // Remove standard output
    }

    virtual ~ColGenSolverCplex() { }

    void add_column(std::vector<int> rows, Cost c)
    {
        IloNumColumn col = obj_(c);
        for (int i: rows)
            col += range_[i](1);
        vars_.push_back(IloNumVar(col, 0, 1, ILOFLOAT));
        model_.add(vars_.back());
    }

    void solve() { cplex_.solve(); }

    double objective() { return cplex_.getObjValue(); }

    std::vector<double>& dual_solution()
    {
        cplex_.getDuals(dual_sol_, range_);
        for (int row = 0; row < m_ + n_; ++ row)
            dual_sol_vd_[row] = dual_sol_[row];
        return dual_sol_vd_;
    }

    std::vector<double> solution()
    {
        std::vector<double> sol(vars_.size());
        for (int i = 0; i < (int)vars_.size(); ++i)
            sol[i] = cplex_.getValue(vars_[i]);
        return sol;
    }

private:

    AgentIdx m_;
    ItemIdx n_;
    IloEnv env_;
    IloModel model_;
    IloObjective obj_;
    IloRangeArray range_;
    IloCplex cplex_;
    std::vector<IloNumVar> vars_;
    IloNumArray dual_sol_;
    std::vector<double> dual_sol_vd_;

};

#endif

ColGenOutput& ColGenOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "Iterations", it);
    VER(info, "Iterations: " << it << std::endl);
    PUT(info, "Algorithm", "AddedColumns", added_column_number);
    VER(info, "Added columns: " << added_column_number << std::endl);
    return *this;
}

void add_column(const Instance& ins,
        ColGenOptionalParameters& p,
        ColGenSolver& solver,
        std::vector<std::vector<std::vector<ItemIdx>>>* columns,
        AgentIdx i,
        ColIdx col_idx,
        std::vector<std::pair<AgentIdx, ColIdx>>& col_indices,
        const std::vector<int>& agent_rows,
        const std::vector<int>& item_rows)
{
    std::vector<int> rows;
    rows.push_back(agent_rows[i]);
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
        if (item_rows[j] == -1) {
            t -= ins.alternative(k).w;
            continue;
        }

        c += ins.alternative(k).c;
        w += ins.alternative(k).w;
        rows.push_back(item_rows[j]);
    }
    col_indices.push_back({i, col_idx});

    solver.add_column(rows, c);
}

ColGenOutput gap::lb_colgen(const Instance& ins, ColGenOptionalParameters p)
{
    VER(p.info, "*** columngeneration " << p.solver << " ***" << std::endl);
    ColGenOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    std::vector<int> agent_row(m, -2);
    int row_idx = 0;
    for (AgentIdx i=0; i<m; ++i) {
        if (p.fixed_agents != NULL && (*p.fixed_agents)[i] == 1)
            continue;
        agent_row[i] = row_idx;
        row_idx++;
    }
    AgentIdx agent_constraint_number = row_idx;

    std::vector<int> item_row(n, -2);
    for (ItemIdx j=0; j<n; ++j) {
        for (AgentIdx i=0; i<m; ++i) {
            if (p.fixed_alt != NULL && (*p.fixed_alt)[ins.alternative_index(j, i)] == 1) {
                item_row[j] = -1;
                break;
            }
        }
        if (item_row[j] == -2) {
            item_row[j] = row_idx;
            row_idx++;
        }
    }
    ItemIdx item_constraint_number = row_idx - agent_constraint_number;

    // Initialize solver
    std::unique_ptr<ColGenSolver> solver = NULL;
#if CPLEX_FOUND
    if (p.solver == "cplex")
        solver = std::unique_ptr<ColGenSolver>(new ColGenSolverCplex(agent_constraint_number, item_constraint_number));
#endif
#if COINOR_FOUND
    //if (solver_str == "clp")
        //solver = std::unique_ptr<ColGenSolver>(new ColGenSolverClp(agent_constraint_number, item_constraint_number));
#endif
    if (solver == NULL)
        return output.algorithm_end(p.info);

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
    std::vector<int> rows(m + n);
    std::iota(rows.begin(), rows.begin() + n, m);
    solver->add_column(rows, 10 * ins.bound());
    output.column_indices.push_back({-1, -1});

    std::vector<std::vector<std::vector<ItemIdx>>>* columns;
    if (p.columns == NULL) {
        output.columns.resize(m);
        columns = &output.columns;
    } else {
        columns = p.columns;
        for (AgentIdx i=0; i<m; ++i)
            for (ColIdx col_idx = 0; col_idx < (ColIdx)(*columns)[i].size(); ++ col_idx)
                add_column(ins, p, *solver, columns, i, col_idx, output.column_indices, agent_row, item_row);
    }

    bool found = true;
    Weight mult = 100000;
    while (found) {
        output.it++;

        // Solve LP
        solver->solve();
        VER(p.info,
                "It " << std::setw(8) << output.it
                << " | T " << std::setw(10) << p.info.elapsed_time()
                << " | C " << std::setw(10) << solver->objective()
                << " | COL " << std::setw(10) << output.added_column_number
                << std::endl);
        std::vector<double>& dual_sol = solver->dual_solution();

        // Find and add new columns
        found = false;
        for (AgentIdx i=0; i<m; ++i) {
            if (agent_row[i] < 0)
                continue;
            ins_kp.clear();
            ins_kp.set_capacity(kp_capacities[i]);
            Cost rc_ub = std::ceil((mult * (- dual_sol[i])));
            ItemIdx j_kp = 0;
            for (ItemIdx j=0; j<n; ++j) {
                AltIdx k = ins.alternative_index(j, i);
                const Alternative& a = ins.alternative(k);
                knapsack::Profit profit = std::floor(mult * dual_sol[item_row[j]]) - std::ceil(mult * a.c);
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
            add_column(ins, p, *solver, columns, i, (*columns)[i].size() - 1, output.column_indices, agent_row, item_row);
            output.added_column_number++;
        }
    }

    // Compute the bound
    // Solve LP
    solver->solve();
    std::vector<double>& dual_sol = solver->dual_solution();

    Cost lb = std::floor(mult * solver->objective());
    for (AgentIdx i=0; i<m; ++i) {
        if (agent_row[i] < 0)
            continue;
        ins_kp.clear();
        ins_kp.set_capacity(kp_capacities[i]);
        Cost rc_lb = std::floor((mult * (- dual_sol[i])));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins.alternative_index(j, i);
            const Alternative& a = ins.alternative(k);
            knapsack::Profit profit = std::ceil(mult * dual_sol[item_row[j]]) - std::floor(mult * a.c);
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
    output.solution = solver->solution();
    output.x.resize(ins.alternative_number(), 0);
    for (ColIdx col_idx = 1; col_idx < (int)output.column_indices.size(); ++col_idx) {
        AgentIdx i       = output.column_indices[col_idx].first;
        ColIdx col_idx_2 = output.column_indices[col_idx].second;
        for (ItemIdx j: (*columns)[i][col_idx_2])
            output.x[ins.alternative_index(j, i)] += output.solution[col_idx];
    }

    return output.algorithm_end(p.info);
}

