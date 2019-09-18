#include "gap/lib/algorithms.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#include "gap/lb_linrelax_gurobi/linrelax_gurobi.hpp"
#include "gap/lb_lagrelax_volume/lagrelax_volume.hpp"
#include "gap/lb_lagrelax_bundle/lagrelax_bundle.hpp"
#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
#include "gap/lb_colgen_clp/colgen_clp.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"
#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"
#include "gap/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"
#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"
#include "gap/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"
#include "gap/opt_dip/dip.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/ub_greedy/greedy.hpp"
#include "gap/ub_repair/repair.hpp"
#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_ls_ejectionchain/ls_ejectionchain.hpp"
#include "gap/ub_localsolver/localsolver.hpp"
#include "gap/ub_vdns_simple/vdns_simple.hpp"
#include "gap/ub_vnsbranching_cbc/vnsbranching_cbc.hpp"
#include "gap/ub_vnsbranching_cplex/vnsbranching_cplex.hpp"

#include <map>

using namespace gap;

std::function<void (Instance&, Solution&, Cost&, std::mt19937_64&, Info)> gap::get_algorithm(std::string str)
{
    benchtools::Algorithm algo(str);

    if (algo.name == "") {
        std::cerr << "\033[32m" << "ERROR, missing algorithm." << "\033[0m" << std::endl;
        return [](Instance&, Solution&, Cost&, std::mt19937_64&, Info) { };

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (algo.name == "linrelax_clp") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_linrelax_clp(ins, info);
            lb = res.lb;
        };
#endif
#if GUROBI_FOUND
    } else if (algo.name == "linrelax_gurobi") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_linrelax_gurobi(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "lagrelax_knapsack_volume") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_volume(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "lagrelax_knapsack_bundle") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_bundle(ins, info);
            lb = res.lb;
        };
#endif
#if DLIB_FOUND
    } else if (algo.name == "lagrelax_knapsack_lbfgs") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_lbfgs(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "lagrelax_assignment_volume") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_volume(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "lagrelax_assignment_bundle") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_bundle(ins, info);
            lb = res.lb;
        };
#endif
#if DLIB_FOUND
    } else if (algo.name == "lagrelax_assignment_lbfgs") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_lbfgs(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "colgen_clp") {
        return [algo](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            std::vector<std::vector<std::vector<ItemIdx>>> columns;
            std::vector<AltIdx> fixed_alt(ins.alternative_number());
            std::fill(fixed_alt.begin(), fixed_alt.end(), -1);
            lb_colgen_clp(ColGenClpData{
                    .ins = ins,
                    .lb = lb,
                    .columns = columns,
                    .fixed_alt = fixed_alt,
                    .info = info});
        };
#endif

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (algo.name == "branchandcut_cbc") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            sopt_branchandcut_cbc({
                    .ins = ins,
                    .sol = sol,
                    .lb = lb,
                    .stop_at_first_improvment = false,
                    .info = info,
                    });
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "branchandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_branchandcut_dip(ins, info);
        };
#endif
#if CPLEX_FOUND
    } else if (algo.name == "branchandcut_cplex") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            sopt_branchandcut_cplex({
                    .ins = ins,
                    .sol = sol,
                    .lb = lb,
                    .info = info,
                    });
        };
#endif
#if GUROBI_FOUND
    } else if (algo.name == "branchandcut_gurobi") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            sopt_branchandcut_gurobi({
                    .ins = ins,
                    .sol = sol,
                    .lb = lb,
                    .info = info,
                    });
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "branchandpriceandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_branchandpriceandcut_dip(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "relaxandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_relaxandcut_dip(ins, info);
        };
#endif
#if GECODE_FOUND
    } else if (algo.name == "constraintprogramming_gecode") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
        sopt_constraintprogramming_gecode({
                .ins = ins,
                .sol = sol,
                .lb = lb,
                .info = info,
                });
        };
#endif
#if CPLEX_FOUND
    } else if (algo.name == "constraintprogramming_cplex") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
        sopt_constraintprogramming_cplex({
                .ins = ins,
                .sol = sol,
                .lb = lb,
                .info = info,
                });
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            (void)info;
            dip(ins);
        };
#endif

    /*
     * Upper bounds
     */
    } else if (algo.name == "random") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_random(ins, gen, info);
        };
    } else if (algo.name == "greedy") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = algo.args.find("f");
            std::string des_str = (it == algo.args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_greedy(ins, *f, info);
        };
    } else if (algo.name == "greedyregret") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = algo.args.find("f");
            std::string des_str = (it == algo.args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_greedyregret(ins, *f, info);
        };
    } else if (algo.name == "mthg") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = algo.args.find("f");
            std::string des_str = (it == algo.args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_mthg(ins, *f, info);
        };
    } else if (algo.name == "mthgregret") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = algo.args.find("f");
            std::string des_str = (it == algo.args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_mthgregret(ins, *f, info);
        };
#if COINOR_FOUND
    } else if (algo.name == "repaircombrelax") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = sol_repaircombrelax(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "repairgreedy") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = sol_repairgreedy(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "repairlinrelax_clp") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
            sol = sol_repairlinrelax_clp(ins, linrelax_output, info);
        };
#endif
    } else if (algo.name == "ls_shiftswap") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_ls_shiftswap(LSShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(algo.args));
        };
    } else if (algo.name == "ts_shiftswap") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_ts_shiftswap(TSShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(algo.args));
        };
    } else if (algo.name == "sa_shiftswap") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_sa_shiftswap(SAShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(algo.args));
        };
    } else if (algo.name == "lsfirst_ejectionchain") {
        return [algo](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
        sol = sol_lsfirst_ejectionchain(LSFirstECData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(algo.args));
        };
#if LOCALSOLVER_FOUND
    } else if (algo.name == "localsolver") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = ub_localsolver({ins, sol, info});
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "vdns_simple") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vdns_simple(ins, sol, gen, info);
        };
#endif
#if COINOR_FOUND
    } else if (algo.name == "vnsbranching_cbc") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vnsbranching_cbc(ins, gen, info);
        };
#endif
#if CPLEX_FOUND
    } else if (algo.name == "vnsbranching_cplex") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vnsbranching_cplex(ins, gen, info);
        };
#endif


    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << algo.name << "\033[0m" << std::endl;
        assert(false);
        return [](Instance&, Solution&, Cost&, std::mt19937_64&, Info) { };
    }
}

