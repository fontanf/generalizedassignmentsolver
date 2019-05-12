#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_vdns_simple/vdns_simple.hpp"
//#include "gap/ub_mbastar/mbastar.hpp"

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
        ("format,f", po::value<std::string>(&format), "set input file format (default: knapsack_standard)")
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

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_logfile(logfile)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        .set_timelimit(time_limit)
        .set_onlywriteattheend(true)
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
    if (vstrings[0] == "linrelax_clp") {
        lb_linrelax_clp(ins, info);
    } else if (vstrings[0] == "branchandcut_cbc") {
        sopt_branchandcut_cbc({
                .ins = ins,
                .sol = sol,
                .stop_at_first_improvment = false,
                .info = info,
                });
    } else if (vstrings[0] == "random") {
        sol = sol_random(ins, gen, info);
    } else if (vstrings[0] == "repairlinrelax") {
        LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
        sol = sol_repairlinrelax(ins, linrelax_output, info);
    } else if (vstrings[0] == "lsfirst_shiftswap") {
        sol = sol_lsfirst_shiftswap(LSFirstShiftSwapData{
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
        PRShiftSwapData d({.ins = ins, .gen = gen, .info = info});
        d.alpha = std::vector<double>(ins.agent_number(), 10.0);
        sol = sol_pr_shiftswap(d);
    } else if (vstrings[0] == "vdns_simple") {
        sol = sol_vdns_simple(ins, gen, info);
    /*
    } else if (algorithm == "tabuastar") {
        sol_tabuastar({
                .ins = ins,
                .growth_factor = 1.5,
                .criterion_id = 3,
                .sol_best = sol,
                .gen = gen,
                .info = info});
    } else if (algorithm == "mbastar1") {
        sol_mbastar_1({
                .ins = ins,
                .growth_factor = 1.5,
                .criterion_id = 3,
                .sol_best = sol,
                .gen = gen,
                .info = info});
    } else if (algorithm == "mbastar2") {
        sol_mbastar_2({
                .ins = ins,
                .growth_factor = 1.5,
                .criterion_id = 3,
                .sol_best = sol,
                .gen = gen,
                .info = info});
    */
    }

    info.write_ini(outputfile);
    sol.write_cert(certfile);
    return 0;
}

