#include "gap/ub_vlsn_mbp/vlsn_mbp.hpp"

#include "gap/ub_random/random.hpp"
#include "gap/ub_repair/repair.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

Solution gap::sol_vlsn_mbp(const Instance& ins, std::mt19937_64& gen, Info info)
{
    (void)gen;
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    init_display(info);
    Solution sol_best(ins);

    LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
    Cost lb = linrelax_output.lb;
    Solution sol_curr = sol_repairlinrelax(ins, linrelax_output);
    //Solution sol_curr = sol_random(ins, gen);

    std::stringstream ss;
    sol_best.update(sol_curr, lb, ss, info);

    std::vector<ItemIdx> items(n, 0);
    std::iota(items.begin(), items.end(), 0);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m-2);

    // TODO

    return algorithm_end(sol_best, info);
}

