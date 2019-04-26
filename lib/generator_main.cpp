#include "gap/lib/generator.hpp"

#include <boost/program_options.hpp>

using namespace gap;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options
    GenerateData data;
    std::string output_file = "";
    std::string plot_file = "";
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        (",t", po::value<std::string>(&data.t)->required(), "set instance type (a, b, c, d, e, nn, ns, nc)")
        (",n", po::value<ItemIdx>(&data.n)->required(), "set item number")
        (",m", po::value<AgentIdx>(&data.m), "set agent number")
        (",r", po::value<Weight>(&data.r), "set R")
        (",s", po::value<Seed>(&data.s), "set seed")
        (",o", po::value<std::string>(&output_file), "set output file")
        (",p", po::value<std::string>(&plot_file), "set plot file")
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

    std::cout << data << std::endl;
    Instance ins = generate(data);

    if (output_file != "")
        ins.write(output_file);
    if (plot_file != "")
        ins.plot(plot_file);

    return 0;
}

