#include "generalizedassignmentsolver/algorithms/algorithms.hpp"

#include <boost/program_options.hpp>

using namespace generalizedassignmentsolver;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options

    std::string algorithm = "branchandcut_cbc";
    std::string instance_path = "";
    std::string format = "orlibrary";
    std::string output_path = "";
    std::string initial_solution_path = "";
    std::string certificate_path = "";
    std::string log_path = "";
    int loglevelmax = 999;
    int seed = 0;
    double time_limit = std::numeric_limits<double>::infinity();

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::string>(&algorithm), "set algorithm")
        ("input,i", po::value<std::string>(&instance_path)->required(), "set input file (required)")
        ("format,f", po::value<std::string>(&format), "set input file format (default: orlibrary)")
        ("initsol", po::value<std::string>(&initial_solution_path), "set initial solution file")
        ("output,o", po::value<std::string>(&output_path), "set output file")
        ("cert,c", po::value<std::string>(&certificate_path), "set certificate file")
        ("time-limit,t", po::value<double>(&time_limit), "Time limit in seconds\n  ex: 3600")
        ("seed,s", po::value<int>(&seed), "seed")
        ("verbose,v", "")
        ("log,l", po::value<std::string>(&log_path), "set log file")
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

    // Run algorithm

    std::mt19937_64 gen(seed);

    Instance instance(instance_path, format);

    Info info = Info()
        .set_verbose(vm.count("verbose"))
        .set_timelimit(time_limit)
        .set_certfile(certificate_path)
        .set_outputfile(output_path)
        .set_onlywriteattheend(false)
        .set_logfile(log_path)
        .set_log2stderr(vm.count("log2stderr"))
        .set_loglevelmax(loglevelmax)
        ;

    run(algorithm, instance, gen, info);

    return 0;
}

