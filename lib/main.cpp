#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#include "gap/lb_lagrelax_volume/lagrelax_volume.hpp"
#include "gap/lb_lagrelax_bundle/lagrelax_bundle.hpp"
#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"
#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"
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
#include "gap/ub_vlsn_mbp/vlsn_mbp.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
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

    if (!boost::filesystem::exists(instancefile)) {
        std::cerr << instancefile << ": file not found." << std::endl;
        return 1;
    }
    for (std::string file: {certfile, outputfile}) {
        if (file == "")
            continue;
        boost::filesystem::path p(file);
        if (p.parent_path() != "")
            boost::filesystem::create_directories(p.parent_path());
    }

    Instance ins(instancefile, format);
    Solution sol(ins);
    if (initsolfile != "")
        sol.read(initsolfile);

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_logfile(logfile)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        .set_timelimit(time_limit)
        .set_onlywriteattheend(false)
        .set_certfile(certfile)
        .set_outputfile(outputfile);

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
    /*
     * Lower bounds
     */
    if (vstrings[0] == "") {
#if COINOR_FOUND
    } else if (vstrings[0] == "linrelax_clp") {
        lb_linrelax_clp(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_volume") {
        lb_lagrelax_knapsack_volume(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_bundle") {
        lb_lagrelax_knapsack_bundle(ins, info);
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_knapsack_lbfgs") {
        lb_lagrelax_knapsack_lbfgs(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_volume") {
        lb_lagrelax_assignment_volume(ins, info);
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_bundle") {
        lb_lagrelax_assignment_bundle(ins, info);
#endif
#if DLIB_FOUND
    } else if (vstrings[0] == "lagrelax_assignment_lbfgs") {
        lb_lagrelax_assignment_lbfgs(ins, info);
#endif
    /*
     * Exact
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
        info.set_onlywriteattheend(false);
        sopt_branchandcut_cplex({
                .ins = ins,
                .sol = sol,
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
#endif
#if COINOR_FOUND
    } else if (vstrings[0] == "vdns_simple") {
        info.set_onlywriteattheend(false);
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
#if CPLEX_FOUND
    } else if (vstrings[0] == "vlsn_mbp") {
        sol = sol_vlsn_mbp(ins, sol, gen, info);
#endif
    } else {
        std::cout << "unknown algorithm" << std::endl;
    }

    info.write_ini(outputfile);
    sol.write_cert(certfile);
    return 0;
}

