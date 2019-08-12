#include "gap/lib/algorithms.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#include "gap/lb_linrelax_gurobi/linrelax_gurobi.hpp"
#include "gap/lb_lagrelax_volume/lagrelax_volume.hpp"
#include "gap/lb_lagrelax_bundle/lagrelax_bundle.hpp"
#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
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
    std::stringstream ss(str);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    std::map<std::string, std::string> args;
    for (auto it=std::next(vstrings.begin());
            it!=vstrings.end() && std::next(it)!=vstrings.end();
            it=std::next(std::next(it)))
        args[*it] = *std::next(it);

    if (vstrings[0] == "") {
        std::cerr << "\033[32m" << "ERROR, missing algorithm." << "\033[0m" << std::endl;
        return [](Instance&, Solution&, Cost&, std::mt19937_64&, Info) { };

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (vstrings[0] == "linrelax_clp") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_linrelax_clp(ins, info);
            lb = res.lb;
        };
#endif
#if GUROBI_FOUND
    } else if (vstrings[0] == "linrelax_gurobi") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_linrelax_gurobi(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_volume") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_volume(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_bundle") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_bundle(ins, info);
            lb = res.lb;
        };
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_lbfgs") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_knapsack_lbfgs(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_volume") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_volume(ins, info);
            lb = res.lb;
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_bundle") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_bundle(ins, info);
            lb = res.lb;
        };
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_lbfgs") {
        return [](Instance& ins, Solution&, Cost& lb, std::mt19937_64&, Info info) {
            auto res = lb_lagrelax_assignment_lbfgs(ins, info);
            lb = res.lb;
        };
#endif

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (vstrings[0] == "branchandcut_cbc") {
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
    } else if (vstrings[0] == "branchandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_branchandcut_dip(ins, info);
        };
#endif
#if CPLEX_FOUND
    } else if (vstrings[0] == "branchandcut_cplex") {
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
    } else if (vstrings[0] == "branchandcut_gurobi") {
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
    } else if (vstrings[0] == "branchandpriceandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_branchandpriceandcut_dip(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "relaxandcut_dip") {
        return [](Instance& ins, Solution& sol, Cost& lb, std::mt19937_64&, Info info) {
            (void)sol;
            (void)lb;
            sopt_relaxandcut_dip(ins, info);
        };
#endif
#if GECODE_FOUND
    } else if (vstrings[0] == "constraintprogramming_gecode") {
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
    } else if (vstrings[0] == "constraintprogramming_cplex") {
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
    } else if (vstrings[0] == "dip") {
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
    } else if (vstrings[0] == "random") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_random(ins, gen, info);
        };
    } else if (vstrings[0] == "greedy") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = args.find("f");
            std::string des_str = (it == args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_greedy(ins, *f, info);
        };
    } else if (vstrings[0] == "greedyregret") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = args.find("f");
            std::string des_str = (it == args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_greedyregret(ins, *f, info);
        };
    } else if (vstrings[0] == "mthg") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = args.find("f");
            std::string des_str = (it == args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_mthg(ins, *f, info);
        };
    } else if (vstrings[0] == "mthgregret") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            auto it = args.find("f");
            std::string des_str = (it == args.end())? "cij": it->second;
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            sol = sol_mthgregret(ins, *f, info);
        };
#if COINOR_FOUND
    } else if (vstrings[0] == "repaircombrelax") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = sol_repaircombrelax(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "repairgreedy") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = sol_repairgreedy(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "repairlinrelax_clp") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
            sol = sol_repairlinrelax_clp(ins, linrelax_output, info);
        };
#endif
    } else if (vstrings[0] == "lsfirst_shift") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_lsfirst_shift(LSFirstShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "lsfirst_shiftswap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_lsfirst_shiftswap(LSFirstShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "lsfirst_shift_swap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_lsfirst_shift_swap(LSFirstShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "lsbest_shiftswap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_lsbest_shiftswap(LSBestShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "ts_shiftswap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_ts_shiftswap(TSShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "sa_shiftswap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_sa_shiftswap(SAShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "pr_shiftswap") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_pr_shiftswap(PRShiftSwapData{
                    .ins = ins,
                    .gen = gen,
                    .info = info
                    }.set_params(args));
        };
    } else if (vstrings[0] == "lsfirst_ejectionchain") {
        return [args](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
        sol = sol_lsfirst_ejectionchain(LSFirstECData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
        };
#if LOCALSOLVER_FOUND
    } else if (vstrings[0] == "localsolver") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64&, Info info) {
            sol = ub_localsolver({ins, sol, info});
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "vdns_simple") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vdns_simple(ins, sol, gen, info);
        };
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "vnsbranching_cbc") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vnsbranching_cbc(ins, gen, info);
        };
#endif
#if CPLEX_FOUND
    } else if (vstrings[0] == "vnsbranching_cplex") {
        return [](Instance& ins, Solution& sol, Cost&, std::mt19937_64& gen, Info info) {
            sol = sol_vnsbranching_cplex(ins, gen, info);
        };
#endif


    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << vstrings[0] << "\033[0m" << std::endl;
        assert(false);
        return [](Instance&, Solution&, Cost&, std::mt19937_64&, Info) { };
    }
}

