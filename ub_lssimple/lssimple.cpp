#include "gap/ub_lssimple/lssimple.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/opt_milp/milp.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>

using namespace gap;

bool move_gap(const Instance& ins, Solution& sol, AgentIdx m, ItemIdx n,
        std::vector<ItemIdx>& items, std::vector<AgentIdx>& agents,
        std::default_random_engine& gen,
        Info& info)
{
    (void)info;
    if (n != ins.item_number())
        std::shuffle(items.begin(), items.end(), gen);
    if (m != ins.agent_number())
        std::shuffle(agents.begin(), agents.end(), gen);
    std::vector<Weight> c(m, 0);
    for (AgentIdx i=0; i<m; ++i)
        c[i] = ins.capacity(agents[i]);
    Instance ins_tmp(m, n);
    std::vector<AgentIdx> sol_vec;
    Value v = 0;
    std::vector<ItemIdx> pos;
    for (ItemIdx j: items) {
        AgentIdx i = -1;
        for (AgentPos i_pos=0; i_pos<m; ++i_pos) {
            if (agents[i_pos] == sol.agent(j)) {
                i = i_pos;
                break;
            }
        }
        if (i == -1)
            continue;
        if (ins_tmp.item_number() == n) {
            c[i] -= ins.alternative(j, sol.agent(j)).w;
            continue;
        }
        v += ins.alternative(j, sol.agent(j)).v;
        ItemIdx j_tmp = ins_tmp.add_item();
        sol_vec.push_back(i);
        for (AgentPos i_pos=0; i_pos<m; ++i_pos)
            ins_tmp.set_alternative(j_tmp, i_pos,
                    ins.alternative(j, agents[i_pos]).w,
                    ins.alternative(j, agents[i_pos]).v);
        pos.push_back(j);
    }
    for (AgentPos i_pos=0; i_pos<m; ++i_pos)
        ins_tmp.set_capacity(i_pos, c[i_pos]);
    Solution sol_tmp(ins_tmp);
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        sol_tmp.set(j, sol_vec[j]);
    sopt_milp({
            .ins = ins_tmp,
            .sol = sol_tmp,
            .stop_at_first_improvment = true,
            .info = Info().set_timelimit(10),
            });
    if (v <= sol_tmp.value())
        return false;
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        sol.set(pos[j], agents[sol_tmp.agent(j)]);
    return true;
}

bool move_mbp(const Instance& ins, Solution& sol, ItemIdx n,
        std::default_random_engine& gen,
        std::uniform_int_distribution<AgentIdx>& dis_i,
        std::vector<ItemIdx>& items,
        Info& info)
{
    //std::cout << "start sol.value() " << sol.value() << std::endl;
    std::shuffle(items.begin(), items.end(), gen);
    (void)info;
    std::vector<AgentIdx> s(n);
    std::vector<AgentIdx> t(n);
    for (ItemIdx j_pos=0; j_pos<n; ++j_pos) {
        ItemIdx j = items[j_pos];
        AgentIdx i = dis_i(gen);
        if (i >= sol.agent(j))
            i++;
        s[j_pos] = sol.agent(j);
        t[j_pos] = i;
    }

    //std::cout << "s ";
    //for (ItemIdx j=0; j<n; ++j)
        //std::cout << " " << s[j];
    //std::cout << std::endl;
    //std::cout << "t ";
    //for (ItemIdx j=0; j<n; ++j)
        //std::cout << " " << t[j];
    //std::cout << std::endl;

    int numberRows = ins.agent_number();
    int numberColumns = n;
    int numberElements = 2 * n;

    // Matrix data - column ordered
    std::vector<CoinBigIndex> start(numberColumns + 1);
    for (ItemIdx j=0; j<=numberColumns; ++j)
        start[j] = 2 * j;
    std::vector<int> length(numberColumns, 2);

    std::vector<int> rows(numberElements);
    std::vector<double> elements(numberElements);
    for (ItemIdx j=0; j<n; ++j) {
        if (s[j] < t[j]) {
            rows[2 * j]     = s[j];
            rows[2 * j + 1] = t[j];
            elements[2 * j]     =   ins.alternative(items[j], s[j]).w;
            elements[2 * j + 1] = - ins.alternative(items[j], t[j]).w;
        } else {
            rows[2 * j]     = t[j];
            rows[2 * j + 1] = s[j];
            elements[2 * j]     = - ins.alternative(items[j], t[j]).w;
            elements[2 * j + 1] =   ins.alternative(items[j], s[j]).w;
        }
    }
    CoinPackedMatrix matrix(true, numberRows, numberColumns, numberElements,
            elements.data(), rows.data(), start.data(), length.data());

    // Rim data
    std::vector<double> objective(numberColumns);
    Value csum = 0;
    for (ItemIdx j=0; j<numberColumns; ++j) {
        csum += ins.alternative(items[j], t[j]).v;
        objective[j] = ins.alternative(items[j], s[j]).v - ins.alternative(items[j], t[j]).v;
    }
    for (ItemIdx j=numberColumns; j<ins.item_number(); ++j)
        csum += ins.alternative(items[j], sol.agent(items[j])).v;

    std::vector<double> rowUpper(numberRows);
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        rowUpper[i] = ins.capacity(i);
    for (ItemIdx j=0; j<n; ++j)
        rowUpper[t[j]] -= ins.alternative(items[j], t[j]).w;
    for (ItemIdx j=n; j<ins.item_number(); ++j)
        rowUpper[sol.agent(items[j])] -= ins.alternative(items[j], sol.agent(items[j])).w;
    std::vector<double> colLower(numberColumns, 0);
    std::vector<double> colUpper(numberColumns, 1);
    OsiCbcSolverInterface solver1;

    // Reduce printout
    solver1.getModelPtr()->setLogLevel(0);
    solver1.messageHandler()->setLogLevel(0);

    // Load problem
    solver1.loadProblem(matrix, colLower.data(), colUpper.data(),
              objective.data(), 0, rowUpper.data());

    // Mark integer
    for (ItemIdx j=0; j<numberColumns; ++j)
        solver1.setInteger(j);

    // Solve
    solver1.initialSolve();

    // Pass data and solver to CbcModel
    CbcModel model(solver1);
    model.setMaximumSeconds(1);

    // Add initial solution
    Weight v = 0;
    for (ItemIdx j=0; j<n; ++j)
        v += ins.alternative(items[j], s[j]).v - ins.alternative(items[j], t[j]).v;
    //std::cout << "v " << v << std::endl;
    std::vector<double> sol_init(numberColumns, 1);
    model.setBestSolution(sol_init.data(), numberColumns, v);
    model.setMaximumSolutions(1);

    // Reduce printout
    model.setLogLevel(0);
    model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

    // Do complete search
    model.branchAndBound();

    // Get solution
    const double *solution = model.solver()->getColSolution();
    for (ItemIdx j=0; j<numberColumns; ++j) {
        if (solution[j] < 0.5)
            sol.set(items[j], t[j]);
    }
    //std::cout << "csum " << csum << " v " << v << " csum+v " << csum + v << std::endl;
    //std::cout << "obj val " << model.solver()->getObjValue() << std::endl;
    //std::cout << "new val " << csum + model.solver()->getObjValue() << std::endl;
    //std::cout << "end sol.value() " << sol.value() << " n " << sol.item_number() << " wf " << sol.feasible() << std::endl;

    return (csum + v > sol.value());
}

Solution gap::sol_lssimple(const Instance& ins, Solution& sol, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    std::vector<ItemIdx> items(ins.item_number(), 0);
    std::vector<AgentIdx> agents(ins.agent_number(), 0);
    std::iota(items.begin(), items.end(), 0);
    std::iota(agents.begin(), agents.end(), 0);
    Solution sol_best = sol;
    init_display(sol_best, lb, info);

    std::default_random_engine gen(0);
    for (Cpt it=0, k=0, m_max=2; info.check_time(); ++it, ++k) {
        if (k > ins.agent_number()) {
            m_max++;
            k = 0;
        }
        for (AgentIdx m=2; m<=m_max; ++m) {
            for (AgentIdx l=0; l<=(m_max-m+1); ++l) {
                if (move_gap(ins, sol, m, ins.item_number(), items, agents, gen, info)) {
                    std::stringstream ss;
                    ss << "it " << it << " gap m " << m << " n " << ins.item_number();
                    sol_best.update(sol, lb, ss, info);
                    k = 0;
                    goto end;
                }
            }
        }
end:;
    }
    return algorithm_end(sol, info);
}

