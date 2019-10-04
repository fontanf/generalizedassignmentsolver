#include "gap/lib/algorithms.hpp"

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

    std::mt19937_64 gen(seed);
    auto func = get_algorithm(algorithm);

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

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_timelimit(time_limit)
        .set_certfile(certfile)
        .set_outputfile(outputfile)
        .set_onlywriteattheend(true)
        .set_logfile(logfile)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        ;

    Output output = func(ins, gen, info);

    // Check

    if (output.solution.feasible() && output.solution.cost() < lb0)
        std::cerr << "\033[31m" << "WARNING, previous lower bound was wrong." << "\033[0m" << std::endl;
    if (sol0.feasible() && output.lower_bound > sol0.cost()) {
        std::cerr << "\033[31m" << "ERROR, lower bound is greater upper bound." << "\033[0m" << std::endl;
        assert(false);
        return 1;
    }

    // Update best solution and bound

    if (vm.count("update")) {
        if (output.solution.feasible() && (!sol0.feasible() || sol0.cost() > output.solution.cost())) {
            std::cerr << "\033[32m" << "New upper bound found." << "\033[0m" << std::endl;
            output.solution.write_cert(instancefile + ".sol");
        }
        if (lb0 < output.lower_bound) {
            std::cerr << "\033[32m" << "New lower bound found." << "\033[0m" << std::endl;
            std::ofstream f_opt(instancefile + ".bound");
            f_opt << output.lower_bound << std::endl;
        }
    }

    return 0;
}

