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
        const std::vector<ItemIdx>& items, const std::vector<AgentIdx>& agents,
        Info& info)
{
    (void)info;
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
            .info = Info(),
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

bool move_gap_first(const Instance& ins, Solution& sol,
        const std::vector<ItemIdx>& items,
        std::vector<std::vector<AgentIdx>>& agents,
        std::default_random_engine& gen,
        Info& info)
{
    std::shuffle(agents.begin(), agents.end(), gen);
    for (std::vector<AgentIdx>& ag: agents) {
        if (move_gap(ins, sol, ag.size(), ins.item_number(), items, ag, info))
            return true;
        if (!info.check_time())
            return false;
    }
    return false;
}

Solution gap::sol_vdns_simple(const Instance& ins, std::default_random_engine& gen, Info info)
{
    Solution sol = sol_random(ins, gen);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    std::vector<ItemIdx> items(ins.item_number(), 0);
    std::iota(items.begin(), items.end(), 0);
    Solution sol_best(ins);
    init_display(info);
    std::stringstream ss;
    sol_best.update(sol, 0, ss, info);

    for (AgentIdx m=2; m<=ins.agent_number() && info.check_time(); ++m) {

        std::vector<std::vector<AgentIdx>> agents;
        std::vector<AgentIdx> vec(m);
        std::iota(vec.begin(), vec.end(), 0);
        while (vec[0] <= ins.agent_number() - m) {
            while (vec.back() < ins.agent_number()) {
                agents.push_back(vec);
                vec.back()++;
            }
            AgentIdx i = m - 1;
            while (i >= 0 && vec[i] >= ins.agent_number() - m + i)
                i--;
            if (i == -1)
                break;
            vec[i]++;
            for (AgentIdx i2=i+1; i2<m; ++i2)
                vec[i2] = vec[i2 - 1] + 1;
        }

        while (move_gap_first(ins, sol, items, agents, gen, info)) {
            std::stringstream ss;
            ss << " gap m " << m << " n " << ins.item_number();
            sol_best.update(sol, lb, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

