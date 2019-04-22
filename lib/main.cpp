#include "gap/opt_milp/milp.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/ub_lssimple/lssimple.hpp"

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
    double time_limit = std::numeric_limits<double>::infinity();

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::string>(&algorithm), "set algorithm")
        ("input,i", po::value<std::string>(&instancefile)->required(), "set input file (required)")
        ("format,f", po::value<std::string>(&format), "set input file format (default: knapsack_standard)")
        ("output,o", po::value<std::string>(&outputfile), "set output file")
        ("cert,c", po::value<std::string>(&certfile), "set certificate file")
        (",t", po::value<double>(&time_limit), "Time limit in seconds\n  ex: 3600")
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

    Instance ins(instancefile, format);
    Solution sol(ins);

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_logfile(logfile)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        .set_timelimit(time_limit)
        .set_outputfile(outputfile);

    if (algorithm == "milp") {
        sopt_milp(ins, sol, info);
    } else if (algorithm == "random") {
        sol = sol_random(ins, info);
    } else if (algorithm == "lssimple") {
        sol_lssimple(ins, sol, info);
    }

    info.write_ini(outputfile);
    sol.write_cert(certfile);
    return 0;
}

