#include "gap/opt_babposta/babposta.hpp"

#include "gap/ub_lagrangian/lagrangian.hpp"

#define DBG(x)
//#define DBG(x) x

using namespace gap;

std::vector<std::vector<Value>> compute_reduced_costs(SubInstance& sub, LagOut& lagout)
{
    DBG(std::cout << "COMPUTEREDUCEDCOSTS..." << std::endl;)
    const Instance& ins = sub.instance();
    std::vector<std::vector<Value>> costs(ins.item_number(), std::vector<Value>(ins.agent_number(), -1));

    for (AgentIdx i=0; i<ins.agent_number(); ++i) {

        std::vector<ItemIdx> idx(ins.item_number());
        ItemIdx jj = 0;
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (sub.reduced(j, i) == -1) {
                idx[j] = -1;
            } else {
                idx[j] = jj;
                jj++;
            }
        }

        DBG(std::cout << "AGENT " << i << std::endl;)
        Weight  c = sub.capacity(i);

        DBG(std::cout << "COMPUTE F" << std::endl;)
        ItemIdx n = sub.agent_alternative_number(i);
        //std::cout << "N " << n << std::endl;
        std::vector<std::vector<Value>> f(n+1, std::vector<Value>(c+1, 0));
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            AltIdx k = ins.alternative_index(j, i);
            if (sub.reduced(k) == -1)
                continue;
            Weight wj = ins.alternative(j, i).w;
            Value pj = ins.alternative(j, i).v - lagout.mult[j];
            ItemIdx jj = idx[j]+1;
            //std::cout << "JJ " << jj << " J " << j << std::endl;
            for (Weight w=0; w<=c; ++w) {
                f[jj][w] = f[jj-1][w];
                if (w >= wj) {
                    Value v = f[jj-1][w-wj] + pj;
                    if (f[jj][w] < v)
                        f[jj][w] = v;
                }
            }
        }
        Value zi = f[n][c];
        DBG(std::cout << "ZI " << zi << std::endl;)
        DBG(
        for (ItemIdx jj=0; jj<=n; ++jj) {
            for (Weight w=0; w<=c; ++w)
                std::cout << f[jj][w] << " " << std::flush;
            std::cout << std::endl;
        }
        )

        DBG(std::cout << "COMPUTE G" << std::endl;)
        std::vector<std::vector<Value>> g(n+1, std::vector<Value>(c+1, 0));
        for (ItemIdx j=ins.item_number()-1; j>=0; --j) {
            AltIdx k = ins.alternative_index(j, i);
            if (sub.reduced(k) == -1)
                continue;
            Weight wj = ins.alternative(j, i).w;
            Value pj = ins.alternative(j, i).v - lagout.mult[j];
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
                        Value v = g[jj+1][w+wj] + pj;
                        if (g[jj][w] < v)
                            g[jj][w] = v;
                    }
                }
            }
        }
        DBG(
        for (ItemIdx jj=n; jj>=0; --jj) {
            for (Weight w=0; w<=c; ++w)
                std::cout << g[jj][w] << " " << std::flush;
            std::cout << std::endl;
        }
        )

        DBG(std::cout << "COMPUTE COSTS" << std::endl;)
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            AltIdx k = ins.alternative_index(j, i);
            if (sub.reduced(k) == -1)
                continue;
            ItemIdx jj = idx[j];
            Value u = 0;
            if (lagout.xji[j][i] == 0) {
                Weight wj = ins.alternative(k).w;
                if (wj > sub.capacity(i)) {
                    u = 0;
                } else {
                    for (Weight w=0; w<=c-wj; ++w) {
                        Value p = f[jj][w] + g[jj+1][w+wj];
                        if (u < p)
                            u = p;
                    }
                    u += ins.alternative(k).v - lagout.mult[j];
                }
            } else {
                for (Weight w=0; w<=c; ++w) {
                    Value p = f[jj][w] + g[jj+1][w];
                    if (u < p)
                        u = p;
                }
            }
            costs[j][i] = zi - u;
            //if (costs[j][i] < 0) {
                //std::cout << "I " << i << " J " << j << std::endl;
                //std::cout << ins.alternative(k) << std::endl;
                //std::cout << " ZI " << zi << " U " << u << std::endl;
                //assert(false);
            //}
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

bool fix_variable(SubInstance& sub, Value z, LagOut& lagout, std::vector<std::vector<Value>> costs)
{
    DBG(std::cout << "FIXVARIABLE..." << std::endl;)
    const Instance& ins = sub.instance();

    DBG(std::cout << "COMPUTE CSUM AND X" << std::endl;)
    std::vector<Value> csum(ins.item_number(), 0);
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (sub.reduced(j, i) == -1)
                continue;
            if (lagout.xji[j][i] == 1)
                csum[j] += costs[j][i];
        }
    }

    DBG(
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        std::cout << csum[j] << " " << std::flush;
    std::cout << std::endl;
    )

    DBG(std::cout << "1" << std::endl;)
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (sub.reduced(j, i) == -1)
                continue;
            // If xij == 0
            // If xij were to take value 1, then all xkj, k != i would take value 0
            if (lagout.xji[j][i] == 0) {
                //std::cout << "U " << lagout.u << " C " << costs[j][i] << " CSUM " << csum[j] << " Z " << z << std::endl;
                if (lagout.u - costs[j][i] - csum[j] < z)
                    sub.remove(j, i);
            // If xij == 1 and xkj == 0 for all k != j
            // If xij were to take value 0, then at least one xkj has to take value 1
            } else {
                if (lagout.xj[j] == 1) { // xij is the only xkj with value 1
                    if (sub.item_alternative_number(j) == 1) {
                        sub.set(j, i);
                    } else {
                        Value cmin = -1;
                        for (AgentIdx ii=0; ii<ins.agent_number(); ++ii) {
                            if (ii == i || sub.reduced(j, ii) == -1)
                                continue;
                            if (cmin == -1 || cmin > costs[j][ii])
                                cmin = costs[j][ii];
                        }
                        //std::cout << "U " << lagout.u << " C " << costs[j][i] << " CMIN " << cmin << " Z " << z << std::endl;
                        assert(cmin != -1);
                        if (lagout.u - costs[j][i] - cmin < z)
                            sub.set(j, i);
                    }
                }
            }
        }
    }
    //std::cout << sub.reduced_solution()->item_number() << std::endl;
    //for (ItemIdx j=0; j<ins.item_number(); ++j)
        //std::cout << "J " << " REMALT " << sub.item_alternative_number(j) << std::endl;

    DBG(std::cout << "2" << std::endl;)
    // If xkj is fixed to 0 for all k != i, then xij can be fixed to 1
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sub.item_alternative_number(j) != 1)
            continue;
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
        Value cmin = -1;
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative(j, i).idx;
            if (sub.reduced(k) == -1)
                continue;
            if (cmin == -1 || cmin < costs[j][i])
                cmin = costs[j][i];
        }
        //std::cout << "U " << lagout.u << " CMIN " << cmin << " Z " << z << std::endl;
        if (lagout.u - cmin < z) {
            DBG(std::cout << "FIXVARIABLE... NO FEASIBLE ALT FOR ITEM " << j << std::endl;)
            return false;
        }
    }

    DBG(std::cout << "4" << std::endl;)
    // If xij is fixed to 0 for all i, then current node can be culled
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sub.reduced_solution()->agent(j) == -1
                && sub.item_alternative_number(j) == 0) {
            DBG(std::cout << "FIXVARIABLE... END NO ALT FOR ITEM " << j << std::endl;)
            return false;
        }
    }

    DBG(std::cout << "FIXVARIABLE... END" << std::endl;)
    return true;
}


struct BabPostaData
{
    BabPostaData(const Instance& ins, Solution* sol, Value z, StateIdx* nodes, LagOut* lo, Info* info):
        sub(ins),
        sol(sol),
        z(z),
        nodes(nodes),
        depth(0),
        lagout(lo),
        mult_init(std::vector<Value>(ins.item_number(), 0)),
        info(info) {  }

    BabPostaData(BabPostaData& d):
        sub(d.sub),
        sol(d.sol),
        z(d.z),
        nodes(d.nodes),
        depth(d.depth+1),
        lagout(NULL),
        mult_init(d.lagout->mult),
        info(d.info) { }
    //BabPostaData(BabPostaData& d): sub(d.sub)
    //{
        //sol = d.sol;
        //z   = d.z;
        //nodes = d.nodes;
        //depth = d.depth+1;
        //lagout = NULL;
        //std::cout << "toto" << std::endl;
        //mult_init = d.lagout->mult;
        //std::cout << "tutu" << std::endl;
        //info = d.info;
    //}

    SubInstance sub;
    Solution* sol;
    Value z;
    StateIdx* nodes;
    ItemIdx depth;
    LagOut* lagout;
    std::vector<Value> mult_init;
    Info* info;
};

void sopt_babposta_rec(BabPostaData& d)
{
    //std::cout << "D " << depth << " F " << sub.reduced_solution()->item_number() << std::endl;
    DBG(std::cout << "BABPOSTAREC..." << std::endl;)
    const Instance& ins = d.sub.instance();
    DBG(
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            std::cout << d.sub.reduced(j, i) << " " << std::flush;
        std::cout << std::endl;
    }
    )

    (*d.nodes)++; // Increment node number

    LagOut lagout = ub_lagrangian(d.sub, 2000, 1, &d.mult_init, NULL);
    if (d.lagout == NULL)
        d.lagout = &lagout;
    DBG(std::cout << "U " << d.lagout->u << std::endl;)
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

    std::vector<std::vector<Value>> costs = compute_reduced_costs(d.sub, *d.lagout);
    bool b = fix_variable(d.sub, d.z, *d.lagout, costs); // Variable fixing procedure
    if (!b) {
        DBG(std::cout << "BABPOSTAREC... END UNFEASIBLE" << std::endl;)
        return;
    }
    if (d.sub.reduced_solution()->value() == d.z && d.sub.reduced_solution()->feasible()) {
        *d.sol = *d.sub.reduced_solution();
        DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
        return;
    }

    // Find the item to branch on
    ItemIdx j       = -1;
    Value mult_max = -1;
    for (ItemIdx jj=0; jj<ins.item_number(); ++jj) {
        if (d.sub.reduced_solution()->agent(jj) >= 0)
            continue;
        if (j == -1 || d.lagout->mult[jj] > mult_max) {
            j        = jj;
            mult_max = d.lagout->mult[jj];
        }
    }
    if (j == -1) {
        DBG(std::cout << "BABPOSTAREC... END J = -1" << std::endl;)
        return;
    }
    DBG(std::cout << "BRANCH ON " << j << std::endl;)

    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        if (d.sub.capacity(i) < ins.alternative(j, i).w)
            continue;
        DBG(std::cout << "=> AGENT " << i << std::endl;)
        BabPostaData di(d);
        di.sub.set(j, i);
        if (di.sub.reduced_solution()->item_number() == ins.item_number()
                && di.sub.reduced_solution()->value() < d.z)
            continue;
        if (di.sub.reduced_solution()->value() == d.z && di.sub.reduced_solution()->feasible()) {
            *d.sol = *di.sub.reduced_solution();
            DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
            return;
        }
        sopt_babposta_rec(di);
        if (d.sol->item_number() == d.z && d.sol->feasible()) {
            DBG(std::cout << "BABPOSTAREC... END SOL FOUND" << std::endl;)
            return;
        }
    }
    DBG(std::cout << "BABPOSTAREC... END" << std::endl;)
}

Solution gap::sopt_babposta(const Instance& ins, Info* info)
{
    DBG(std::cout << "BABPOSTA..." << std::endl;)
    DBG(std::cout << ins << std::endl;)

    if (ins.agent_number() == 1) {
        Solution sol(ins);
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            sol.set(j, 0);
        DBG(std::cout << "BABPOSTA... END 1 ITEM" << std::endl;)
        if (sol.feasible())
            return sol;
        return Solution(ins);
    }

    Instance ins_ad = ins.adjust();

    SubInstance sub(ins_ad);
    LagOut lagout = ub_lagrangian(sub, 1 << 19, 1, NULL, info);
    Solution sol(ins_ad);
    Value coef = 100000;
    Value z = (lagout.u/coef)*coef;

    while (sol.item_number() != ins_ad.item_number() && z >= 0) {
        if (Info::verbose(info))
            std::cout << "Z " << z << std::flush;
        StateIdx nodes = 0;
        BabPostaData d(ins_ad, &sol, z, &nodes, &lagout, info);
        sopt_babposta_rec(d);
        if (Info::verbose(info))
            std::cout << " NODES " << nodes << std::endl;
        //z--;
        z-=coef;
    }

    Solution sol_orig(ins);
    sol_orig = sol;
    DBG(std::cout << "BABPOSTA... END" << std::endl;)
    return sol_orig;
}

#undef DBG
