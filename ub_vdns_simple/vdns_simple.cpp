#include "gap/ub_vdns_simple/vdns_simple.hpp"
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
            .info = Info().set_timelimit(std::max(1., info.elapsed_time() / 50)),
            });
    if (v <= sol_tmp.value())
        return false;

    //for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        //if (sol.agent(pos[j]) != agents[sol_tmp.agent(j)])
            //std::cout << "item " << items[j] << " prev " << sol.agent(pos[j]) << " new " << agents[sol_tmp.agent(j)] << std::endl;

    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        sol.set(pos[j], agents[sol_tmp.agent(j)]);
    return true;
}

Solution gap::sol_vdns_simple(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    std::vector<ItemIdx> items(ins.item_number(), 0);
    std::vector<AgentIdx> agents(ins.agent_number(), 0);
    std::iota(items.begin(), items.end(), 0);
    std::iota(agents.begin(), agents.end(), 0);
    Solution sol_best = sol;
    init_display(sol_best, lb, info);

    for (Cpt it=0, k=0, m_max=2; info.check_time(); ++it, ++k) {
        if (k > ins.agent_number()) {
            m_max++;
            k = 0;
        }
        for (AgentIdx m=2; m<=m_max; ++m) {
            for (AgentIdx l=0; l<=(m_max-m+1); ++l) {
                if (move_gap(ins, sol, std::min(m, ins.agent_number()), ins.item_number(), items, agents, gen, info)) {
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

