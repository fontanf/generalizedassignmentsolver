#include "gap/opt_milp/milp.hpp"
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
    std::string algorithm = "milp";
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

    std::default_random_engine gen(seed);
    if (algorithm == "milp") {
        sopt_milp({
                .ins = ins,
                .sol = sol,
                .stop_at_first_improvment = false,
                .info = info,
                });
    } else if (algorithm == "random") {
        sol = sol_random(ins, gen, info);
    } else if (algorithm == "dualls_shiftswap") {
        sol = sol_dualls_shiftswap({
                .ins = ins,
                .gen = gen,
                .info = info});
    } else if (algorithm == "lsfirst_shiftswap") {
        sol = sol_lsfirst_shiftswap({
                .ins = ins,
                .gen = gen,
                .alpha = 10,
                .info = info});
    } else if (algorithm == "lsbest_shiftswap") {
        sol = sol_lsbest_shiftswap({
                .ins = ins,
                .gen = gen,
                .alpha = 10,
                .info = info});
    } else if (algorithm == "ts_shiftswap") {
        sol = sol_ts_shiftswap({
                .ins = ins,
                .gen = gen,
                .alpha = 4,
                .info = info});
    } else if (algorithm == "sa_shiftswap") {
        sol = sol_sa_shiftswap({
                .ins = ins,
                .gen = gen,
                .alpha = 10,
                .info = info});
    } else if (algorithm == "pr_shiftswap") {
        sol = sol_pr_shiftswap({
                .ins = ins,
                .gen = gen,
                .alpha = 10,
                .info = info});
    } else if (algorithm == "vdns_simple") {
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

