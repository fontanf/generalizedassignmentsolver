#include "gap/opt_cplex/cplex.hpp"

using namespace gap;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options
    std::string input_data  = "";
    std::string output_file = "";
    std::string cert_file   = "";
    std::string algorithm   = "";
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input-data,i",  po::value<std::string>(&input_data)->required(), "set input data (required)")
        ("output-file,o", po::value<std::string>(&output_file),            "set output file")
        ("cert-file,c",   po::value<std::string>(&cert_file),              "set certificate output file")
        ("verbose,v",                                                      "enable verbosity")
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

    Instance ins(input_data);
    Info info;
    info.verbose(vm.count("verbose"));

    Solution sol = sopt_cplex(ins, &info);

    double t = info.elapsed_time();
    info.pt.put("Solution.Time", t);
    info.pt.put("Solution.OPT", sol.value());
    if (Info::verbose(&info)) {
        std::cout << "---" << std::endl;
        std::cout << ins.print_opt(sol.value()) << std::endl;
        std::cout << "TIME " << t << std::endl;
    }

    info.write_ini(output_file); // Write output file
    sol.write_cert(cert_file); // Write certificate file
    return 0;
}