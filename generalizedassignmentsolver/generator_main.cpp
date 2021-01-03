#include "generalizedassignmentsolver/generator.hpp"

#include <boost/program_options.hpp>

using namespace generalizedassignmentsolver;

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options
    Generator data;
    std::string output_file = "";
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        (",t", po::value<std::string>(&data.t)->required(), "set t")
        (",n", po::value<ItemIdx>(&data.n)->required(), "set n")
        (",m", po::value<AgentIdx>(&data.m), "set m")
        (",r", po::value<Weight>(&data.r), "set r")
        (",x", po::value<double>(&data.x), "set x")
        (",s", po::value<Seed>(&data.s), "set seed")
        (",o", po::value<std::string>(&output_file), "set output file")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;;
        return 1;
    }
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
        std::cout << desc << std::endl;;
        return 1;
    }

    std::cout << data << std::endl;
    Instance ins = data.generate();

    if (output_file != "")
        ins.write(output_file);

    return 0;
}

