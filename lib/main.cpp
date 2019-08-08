#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
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

#include <boost/program_options.hpp>

using namespace gap;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options

    std::string algorithm = "branchandcut_cbc";
    std::string instancefile = "";
    std::string outputfile = "";
    std::string format = "gap_beasley";
    std::string initsolfile = "";
    std::string certfile = "";
    std::string logfile = "";
    int loglevelmax = 999;
    int seed = 0;
    double time_limit = std::numeric_limits<double>::infinity();

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::string>(&algorithm), "set algorithm")
        ("input,i", po::value<std::string>(&instancefile)->required(), "set input file (required)")
        ("format,f", po::value<std::string>(&format), "set input file format (default: gap_beasley)")
        ("initsol", po::value<std::string>(&initsolfile), "set initial solution file")
        ("output,o", po::value<std::string>(&outputfile), "set output file")
        ("cert,c", po::value<std::string>(&certfile), "set certificate file")
        ("time-limit,t", po::value<double>(&time_limit), "Time limit in seconds\n  ex: 3600")
        ("seed,s", po::value<int>(&seed), "seed")
        ("verbose,v", "")
        ("update,u", "")
        ("log,l", po::value<std::string>(&logfile), "set log file")
        ("loglevelmax", po::value<int>(&loglevelmax), "set log max level")
        ("log2stderr", "write log in stderr")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;;
        return 1;
    }
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        return 1;
    }

    Instance ins(instancefile, format);

    // Read current solution and bound

    Solution sol0(ins);
    std::ifstream f_sol(instancefile + ".sol");
    if (f_sol.good())
        sol0 = Solution(ins, instancefile + ".sol");
    f_sol.close();

    Cost lb0 = 0;
    std::ifstream f_bound(instancefile + ".bound");
    if (f_bound.good())
        f_bound >> lb0;
    f_bound.close();

    // Run algorithm

    Solution sol(ins, initsolfile);
    Cost lb = 0;

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_timelimit(time_limit)
        .set_certfile(certfile)
        .set_outputfile(outputfile)
        .set_onlywriteattheend(false)
        .set_logfile(logfile)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        ;

    std::stringstream ss(algorithm);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    std::map<std::string, std::string> args;
    for (auto it=std::next(vstrings.begin());
            it!=vstrings.end() && std::next(it)!=vstrings.end();
            it=std::next(std::next(it)))
        args[*it] = *std::next(it);

    std::mt19937_64 gen(seed);
    if (vstrings[0] == "") {
        std::cout << "missing algorithm" << std::endl;

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (vstrings[0] == "linrelax_clp") {
        auto res = lb_linrelax_clp(ins, info);
        lb = res.lb;
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_volume") {
        auto res = lb_lagrelax_knapsack_volume(ins, info);
        lb = res.lb;
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_bundle") {
        auto res = lb_lagrelax_knapsack_bundle(ins, info);
        lb = res.lb;
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_lbfgs") {
        auto res = lb_lagrelax_knapsack_lbfgs(ins, info);
        lb = res.lb;
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_volume") {
        auto res = lb_lagrelax_assignment_volume(ins, info);
        lb = res.lb;
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_bundle") {
        auto res = lb_lagrelax_assignment_bundle(ins, info);
        lb = res.lb;
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_lbfgs") {
        auto res = lb_lagrelax_assignment_lbfgs(ins, info);
        lb = res.lb;
#endif

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (vstrings[0] == "branchandcut_cbc") {
        sopt_branchandcut_cbc({
                .ins = ins,
                .sol = sol,
                .stop_at_first_improvment = false,
                .info = info,
                });
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "branchandcut_dip") {
        sopt_branchandcut_dip(ins, info);
#endif
#if CPLEX_FOUND
    } else if (vstrings[0] == "branchandcut_cplex") {
        sopt_branchandcut_cplex({
                .ins = ins,
                .sol = sol,
                .lb = lb,
                .info = info,
                });
#endif
#if GUROBI_FOUND
    } else if (vstrings[0] == "branchandcut_gurobi") {
        sopt_branchandcut_gurobi({
                .ins = ins,
                .sol = sol,
                .lb = lb,
                .info = info,
                });
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "branchandpriceandcut_dip") {
        sopt_branchandpriceandcut_dip(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "relaxandcut_dip") {
        sopt_relaxandcut_dip(ins, info);
#endif
#if GECODE_FOUND
    } else if (vstrings[0] == "constraintprogramming_gecode") {
        sopt_constraintprogramming_gecode({
                .ins = ins,
                .sol = sol,
                .lb = lb,
                .info = info,
                });
#endif
#if CPLEX_FOUND
    } else if (vstrings[0] == "constraintprogramming_cplex") {
        sopt_constraintprogramming_cplex({
                .ins = ins,
                .sol = sol,
                .info = info,
                });
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "dip") {
        dip(ins);
#endif

    /*
     * Upper bounds
     */
    } else if (vstrings[0] == "random") {
        sol = sol_random(ins, gen, info);
    } else if (vstrings[0] == "greedy") {
        auto it = args.find("f");
        std::string des_str = (it == args.end())? "cij": it->second;
        std::unique_ptr<Desirability> f = desirability(des_str, ins);
        sol = sol_greedy(ins, *f, info);
    } else if (vstrings[0] == "greedyregret") {
        auto it = args.find("f");
        std::string des_str = (it == args.end())? "cij": it->second;
        std::unique_ptr<Desirability> f = desirability(des_str, ins);
        sol = sol_greedyregret(ins, *f, info);
    } else if (vstrings[0] == "mthg") {
        auto it = args.find("f");
        std::string des_str = (it == args.end())? "cij": it->second;
        std::unique_ptr<Desirability> f = desirability(des_str, ins);
        sol = sol_mthg(ins, *f, info);
    } else if (vstrings[0] == "mthgregret") {
        auto it = args.find("f");
        std::string des_str = (it == args.end())? "cij": it->second;
        std::unique_ptr<Desirability> f = desirability(des_str, ins);
        sol = sol_mthgregret(ins, *f, info);
#if COINOR_FOUND
    } else if (vstrings[0] == "repaircombrelax") {
        sol = sol_repaircombrelax(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "repairgreedy") {
        sol = sol_repairgreedy(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "repairlinrelax") {
        LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
        sol = sol_repairlinrelax(ins, linrelax_output, info);
#endif
    } else if (vstrings[0] == "lsfirst_shift") {
        sol = sol_lsfirst_shift(LSFirstShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "lsfirst_shiftswap") {
        sol = sol_lsfirst_shiftswap(LSFirstShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "lsfirst_shift_swap") {
        sol = sol_lsfirst_shift_swap(LSFirstShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "lsbest_shiftswap") {
        sol = sol_lsbest_shiftswap(LSBestShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "ts_shiftswap") {
        sol = sol_ts_shiftswap(TSShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "sa_shiftswap") {
        sol = sol_sa_shiftswap(SAShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "pr_shiftswap") {
        sol = sol_pr_shiftswap(PRShiftSwapData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
    } else if (vstrings[0] == "lsfirst_ejectionchain") {
        sol = sol_lsfirst_ejectionchain(LSFirstECData{
                .ins = ins,
                .gen = gen,
                .info = info
                }.set_params(args));
#if LOCALSOLVER_FOUND
    } else if (vstrings[0] == "localsolver") {
        sol = ub_localsolver({ins, sol, info});
        is_ub = true;
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "vdns_simple") {
        sol = sol_vdns_simple(ins, sol, gen, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "vnsbranching_cbc") {
        sol = sol_vnsbranching_cbc(ins, gen, info);
#endif
#if CPLEX_FOUND
    } else if (vstrings[0] == "vnsbranching_cplex") {
        sol = sol_vnsbranching_cplex(ins, gen, info);
#endif

    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << algorithm << "\033[0m" << std::endl;
    }

    // Check

    if (sol.feasible() && sol.cost() < lb0)
        std::cerr << "\033[31m" << "WARNING, previous lower bound was wrong." << "\033[0m" << std::endl;
    if (sol0.feasible() && lb > sol0.cost()) {
        std::cerr << "\033[31m" << "ERROR, lower bound is greater upper bound." << "\033[0m" << std::endl;
        assert(false);
        return 1;
    }

    // Update best solution and bound

    if (vm.count("update")) {
        if (!sol0.feasible() || sol0.cost() > sol0.cost()) {
            std::cerr << "\033[32m" << "New upper bound found." << "\033[0m" << std::endl;
            sol.write_cert(instancefile + ".sol");
        }
        if (lb0 < lb) {
            std::cerr << "\033[32m" << "New lower bound found." << "\033[0m" << std::endl;
            std::ofstream f_opt(instancefile + ".bound");
            f_opt << lb;
        }
    }

    info.write_ini(outputfile);
    sol.write_cert(certfile);
    return 0;
}

