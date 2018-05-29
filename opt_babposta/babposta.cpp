#include "gap/opt_babposta/babposta.hpp"

#include "gap/ub_lagrangian/lagrangian.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

//#define DBG(x)
#define DBG(x) x

using namespace gap;

std::vector<std::vector<Profit>> compute_reduced_costs(SubInstance& sub, LagOut& lagout)
{
    DBG(std::cout << "COMPUTEREDUCEDCOSTS..." << std::endl;)
    const Instance& ins = sub.instance();
    std::vector<std::vector<Profit>> costs(ins.item_number(), std::vector<Profit>(ins.agent_number(), -1));

    std::vector<ItemIdx> idx;
    idx.reserve(ins.item_number());
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sub.reduced_solution()->agent(j) >= 0)
            continue;
        idx.push_back(j);
    }
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        DBG(std::cout << "AGENT " << i << std::endl;)
        Weight  c = sub.capacity(i);

        DBG(std::cout << "COMPUTE F" << std::endl;)
        std::vector<std::vector<Profit>> f(idx.size(), std::vector<Profit>(c+1, 0));
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (sub.reduced_solution()->agent(j) >= 0)
                continue;
            Weight wj = ins.alternative(j, i).w;
            Profit pj = ins.alternative(j, i).p - lagout.mult[j];
            ItemIdx jj = idx[j];
            if (idx[jj] == 0) {
                for (Weight w=0; w<=c; ++w) {
                    f[jj][w] = 0;
                    if (w >= wj && pj > 0)
                        f[jj][w] = pj;
                }
            } else {
                for (Weight w=0; w<=c; ++w) {
                    f[jj][w] = f[jj-1][w];
                    if (w >= wj) {
                        Profit v = f[jj-1][w-wj] + pj;
                        if (f[jj][w] < v)
                            f[jj][w] = v;
                    }
                }
            }
        }
        Profit zi = f[idx.back()][c];
        DBG(
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            for (Weight w=0; w<=c; ++w)
                std::cout << f[j][w] << " " << std::flush;
            std::cout << std::endl;
        }
        )

        DBG(std::cout << "COMPUTE G" << std::endl;)
        std::vector<std::vector<Profit>> g(idx.size()+1, std::vector<Profit>(c+1, 0));
        for (ItemIdx j=ins.item_number()-1; j>=0; --j) {
            if (sub.reduced_solution()->agent(j) >= 0)
                continue;
            Weight wj = ins.alternative(j, i).w;
            Profit pj = ins.alternative(j, i).p - lagout.mult[j];
            ItemIdx jj = idx[j];
            if (jj == ins.item_number()-1) {
                for (Weight w=0; w<=c; ++w) {
                    g[jj][w] = 0;
                    if (c - w >= wj && pj > 0)
                        g[jj][w] = pj;
                }
            } else {
                for (Weight w=0; w<=c; ++w) {
                    g[jj][w] = g[jj+1][w];
                    if (c - w >= wj) {
                        Profit v = g[jj+1][w+wj] + pj;
                        if (g[jj][w] < v)
                            g[jj][w] = v;
                    }
                }
            }
        }
        DBG(
        for (ItemIdx j=ins.item_number()-1; j>=0; --j) {
            for (Weight w=0; w<=c; ++w)
                std::cout << g[j][w] << " " << std::flush;
            std::cout << std::endl;
        }
        )

        DBG(std::cout << "COMPUTE COSTS" << std::endl;)
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (sub.reduced_solution()->agent(j) >= 0)
                continue;
            ItemIdx jj = idx[j];
            Profit u = 0;
            if (lagout.xji[j][i] == 0) {
                Weight wj = ins.alternative(j, i).w;
                for (Weight w=0; w<=c-wj; ++w) {
                    Profit p = f[jj][w] + g[jj+1][w+wj];
                    if (u < p)
                        u = p;
                }
                u += ins.alternative(j, i).p - lagout.mult[j];
            } else {
                for (Weight w=0; w<=c; ++w) {
                    Profit p = f[jj][w] + g[jj+1][w];
                    if (u < p)
                        u = p;
                }
            }
            costs[j][i] = zi - u;
            assert(costs[j][i] >= 0);
        }
    }

    DBG(
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            std::cout << costs[j][i] << " " << std::flush;
        std::cout << std::endl;
    }
    )
    DBG(std::cout << "COMPUTEREDUCEDCOSTS... END" << std::endl;)
    return costs;
}

bool fix_variable(SubInstance& sub, Profit z, LagOut& lagout, std::vector<std::vector<Profit>> costs)
{
    DBG(std::cout << "FIXVARIABLE..." << std::endl;)
    const Instance& ins = sub.instance();

    DBG(std::cout << "COMPUTE CSUM AND X" << std::endl;)
    std::vector<Profit> csum(ins.item_number(), 0);
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (sub.reduced(j, i) == -1)
                continue;
            if (lagout.xji[j][i] == 1) {
                csum[j] += costs[j][i];
            }
        }
    }

    DBG(std::cout << "1" << std::endl;)
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (sub.reduced(j, i) == -1)
                continue;
            // If xij == 0
            // If xij were to take value 1, then all xkj, k != i would take value 0
            if (lagout.xji[j][i] == 0) {
                if (lagout.u - costs[j][i] - csum[j] < z)
                    sub.remove(j, i);
            // If xij == 1 and xkj == 0 for all k != j
            // If xij were to take value 0, then at least one xkj has to take value 1
            } else {
                if (lagout.xj[j] == 1) { // xij is the only xkj with value 1
                    Profit cmin = -1;
                    for (AgentIdx ii=0; ii<ins.agent_number(); ++ii) {
                        if (ii == i || (sub.reduced(j, i) == -1))
                            continue;
                        if (cmin == -1 || cmin < costs[j][ii])
                            cmin = costs[j][ii];
                    }
                    assert(cmin != -1);
                    if (lagout.u - costs[j][i] - cmin < z)
                        sub.set(j, i);
                }
            }
        }
    }

    DBG(std::cout << "2" << std::endl;)
    // If xkj is fixed to 0 for all k != i, then xij can be fixed to 1
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sub.item_alternative_number(j) != 1)
            continue;
        std::cout << j << " " << lagout.xj[j] << std::endl;
        assert(lagout.xj[j] <= 1);
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (sub.reduced(j, i) == -1)
                continue;
            if (lagout.xj[j] == 0 && lagout.u - costs[j][i] < z) {
                DBG(std::cout << "FIXVARIABLE... ONLY ALT UNFEASIBLE FOR ITEM " << j << std::endl;)
                return false;
            } else {
                sub.set(j, i);
                break;
            }
        }
    }

    DBG(std::cout << "3" << std::endl;)
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (lagout.xj[j] > 0)
            continue;
        Profit cmin = -1;
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative(j, i).idx;
            if (sub.reduced(k) == -1)
                continue;
            if (cmin == -1 || costs[j][i] < cmin)
                cmin = costs[j][i];
            if (lagout.u - cmin < z) {
                DBG(std::cout << "FIXVARIABLE... NO FEASIBLE ALT FOR ITEM " << j << std::endl;)
                return false;
            }
        }
    }

    DBG(std::cout << "4" << std::endl;)
    // If xij is fixed to 0 for all i, then current node can be culled
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        bool b = false;
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            if (sub.reduced(k) == 0 || sub.reduced_solution()->agent(j) >= 0)
                b = true;
        }
        if (b == false) {
            DBG(std::cout << "FIXVARIABLE... END NO ALT FOR ITEM " << j << std::endl;)
            return false;
        }
    }

    DBG(std::cout << "FIXVARIABLE... END" << std::endl;)
    return true;
}

void sopt_babposta_rec(SubInstance& sub, Profit z, Solution& sol_best, StateIdx& nodes, Info* info)
{
    DBG(std::cout << "BABPOSTAREC..." << std::endl;)
    const Instance& ins = sub.instance();
    DBG(
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            std::cout << sub.reduced(j, i) << " " << std::flush;
        std::cout << std::endl;
    }
    )

    nodes++; // Increment node number

    LagOut lagout = ub_lagrangian(sub);
    DBG(std::cout << "U " << lagout.u << std::endl;)
    DBG(
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        std::cout << "J " << j << " XJ " << lagout.xj[j] << " XIJ " << std::flush;
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            std::cout << lagout.xji[j][i] << " " << std::flush;
        std::cout << std::endl;
    }
    std::cout << "MULT " << std::flush;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        std::cout << lagout.mult[j] << " " << std::flush;
    std::cout << std::endl;
    )
    if (lagout.u == -1) {
        DBG(std::cout << "BABPOSTAREC... END UNFEASIBLE UB" << std::endl;)
        return;
    }

    std::vector<std::vector<Profit>> costs = compute_reduced_costs(sub, lagout);
    bool b = fix_variable(sub, z, lagout, costs); // Variable fixing procedure
    if (!b) {
        DBG(std::cout << "BABPOSTAREC... END UNFEASIBLE" << std::endl;)
        return;
    }
    if (sub.reduced_solution()->profit() == z && sub.reduced_solution()->feasible()) {
        sol_best = *sub.reduced_solution();
        DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
        return;
    }

    // Find the item to branch on
    ItemIdx j       = -1;
    Profit mult_max = -1;
    for (ItemIdx jj=0; jj<ins.item_number(); ++jj) {
        if (sub.reduced_solution()->agent(jj) >= 0)
            continue;
        if (j == -1 || lagout.mult[jj] > mult_max) {
            j        = jj;
            mult_max = lagout.mult[jj];
        }
    }
    DBG(std::cout << "BRANCH ON " << j << std::endl;)

    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        if (sub.capacity(i) < ins.alternative(j, i).w)
            continue;
        DBG(std::cout << "=> AGENT " << i << std::endl;)
        SubInstance subi(sub);
        subi.set(j, i);
        if (subi.reduced_solution()->item_number() == ins.item_number()
                && subi.reduced_solution()->profit() < z)
            continue;
        if (subi.reduced_solution()->profit() == z && subi.reduced_solution()->feasible()) {
            sol_best = *subi.reduced_solution();
            DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
            return;
        }
        sopt_babposta_rec(subi, z, sol_best, nodes, info);
        if (sol_best.item_number() == z && sol_best.feasible()) {
            DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
            return;
        }
    }
    DBG(std::cout << "BABPOSTAREC... END" << std::endl;)
}

Solution gap::sopt_babposta(const Instance& ins, Info* info)
{
    DBG(std::cout << "BABPOSTA..." << std::endl;)

    if (ins.agent_number() == 1) {
        Solution sol(ins);
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            sol.set(j, 0);
        DBG(std::cout << "BABPOSTA... END 1 ITEM" << std::endl;)
        if (sol.feasible())
            return sol;
        return Solution(ins);
    }

    SubInstance sub(ins);
    LagOut lagout = ub_lagrangian(sub);
    Solution sol(ins);
    Profit z = lagout.u;

    while (sol.item_number() != ins.item_number() && z >= 0) {
        DBG(std::cout << "Z " << z << std::endl;)
        StateIdx nodes = 0;
        SubInstance sub_rec(ins);
        sopt_babposta_rec(sub_rec, z, sol, nodes, info);
        z--;
    }

    DBG(std::cout << "BABPOSTA... END" << std::endl;)
    return sol;
}

#undef DBG
