#include "gap/lib/algorithms.hpp"
#include "gap/lib/generator.hpp"
#include "gap/lib/datasets.hpp"

#include <iomanip>
#include <experimental/filesystem>
#include <boost/program_options.hpp>

using namespace gap;

void bench_normal(
        std::string algorithm,
        std::mt19937_64& gen,
        double time_limit)
{
    auto func = get_algorithm(algorithm);

    std::string dir = algorithm;
    while(benchtools::replace(dir, " ", "_"));
    if (time_limit != std::numeric_limits<double>::infinity())
        dir += "_" + std::to_string(time_limit);
    std::experimental::filesystem::create_directory(dir);

    Generator data;
    data.t = "n";
    for (ItemIdx n: {100, 1000, 10000}) {
        data.n = n;
        for (double mx: {0.0, 0.05, 0.1, 0.2, 0.33, 0.5}) {
            data.mx = mx;
            for (Weight r: {1000, 10000, 100000}) {
                data.r = r;
                for (double x: {0.0, 0.2, 0.4, 0.6, 0.8}) {
                    data.x = x;
                    data.s = n + r + n * mx + x * 10;
                    Instance ins = data.generate();
                    std::ostringstream ss;
                    ss << data;
                    std::cout << std::left << std::setw(50) << ss.str() << std::flush;

                    Solution sol(ins);
                    Cost lb = 0;
                    Info info = Info()
                        .set_verbose(false)
                        .set_timelimit(time_limit)
                        .set_onlywriteattheend(true)
                        ;

                    func(ins, sol, lb, gen, info);

                    std::string ub_str = (!sol.feasible())? "inf": std::to_string(sol.cost());
                    std::string lb_str = (lb == ins.bound())? "inf": std::to_string(lb);
                    if ((sol.feasible() && sol.cost() == lb)
                            || (!sol.feasible() && lb == ins.bound()))
                        std::cout << "\033[32m";
                    double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
                    std::cout << std::fixed << std::noshowpoint;
                    std::cout << std::left << std::setw(6) << " | UB";
                    std::cout << std::right << std::setw(10) << ub_str;
                    std::cout << std::left << std::setw(6) << " | LB";
                    std::cout << std::right << std::setw(10) << lb_str;
                    std::cout << std::left << std::setw(8) << " | T (s)";
                    std::cout << std::right << std::setw(10) << t;
                    std::cout << "\033[0m" << std::endl;
                }
            }
        }
    }
}

void bench_hard(
        std::string algorithm,
        std::mt19937_64& gen,
        double time_limit)
{
    auto func = get_algorithm(algorithm);

    std::string dir = algorithm;
    while(benchtools::replace(dir, " ", "_"));
    if (time_limit != std::numeric_limits<double>::infinity())
        dir += "_" + std::to_string(time_limit);
    std::experimental::filesystem::create_directory(dir);

    for (auto it = datasets.begin(); it != datasets.end(); ++it) {
        // Print dataset name centered
        std::string s = "--- " + it->first + " (" + std::to_string(it->second->size()) + ") --- ";
        int pos = (int)((100-s.length())/2);
        for(int i=0; i<pos; i++)
            std::cout << " ";
        std::cout << s << std::endl;

        Dataset<Instance>* d = it->second.get();
        for (InsId i=0; i<d->size(); ++i) {
            Instance ins = d->instance(i);

            std::string s = ins.name();
            benchtools::replace(s, "data", dir);

            std::string outputfile = s + ".ini";
            std::cout << std::left << std::setw(60) << ins.name() << std::flush;

            Solution sol(ins);
            Cost lb = 0;
            Info info = Info()
                .set_verbose(false)
                .set_timelimit(time_limit)
                .set_outputfile(outputfile)
                .set_onlywriteattheend(true)
                ;

            func(ins, sol, lb, gen, info);
            info.write_ini(outputfile);

            std::string ub_str = (!sol.feasible())? "inf": std::to_string(sol.cost());
            std::string lb_str = (lb == ins.bound())? "inf": std::to_string(lb);
            if ((sol.feasible() && sol.cost() == lb)
                    || (!sol.feasible() && lb == ins.bound()))
                std::cout << "\033[32m";
            double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
            std::cout << std::left << std::setw(6) << " | UB";
            std::cout << std::right << std::setw(5) << ub_str;
            std::cout << std::left << std::setw(6) << " | LB";
            std::cout << std::right << std::setw(5) << lb_str;
            std::cout << std::left << std::setw(8) << " | T (s)";
            std::cout << std::right << std::setw(8) << t;
            std::cout << "\033[0m" << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;

    // Parse program options
    std::vector<std::string> algorithms;
    std::vector<std::string> datasets;
    double time_limit = std::numeric_limits<double>::infinity();

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::vector<std::string>>(&algorithms)->multitoken(), "algorithms (bestfitdecreasing, martello...)")
        ("datasets,d", po::value<std::vector<std::string>>(&datasets)->multitoken(), "datasets (normal, hard)")
        ("time-limit,t", po::value<double>(&time_limit), "time limit in seconds")
        ("verbose,v", "")
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

    Seed seed = 0;
    std::mt19937_64 gen(seed);
    //bool verbose = vm.count("verbose");

    for (std::string algorithm: algorithms) {
        std::cout << "*** " << algorithm << " ***" << std::endl;
        for (std::string dataset: datasets) {
            if (dataset == "normal") {
                bench_normal(algorithm, gen, time_limit);
            } else if (dataset == "hard") {
                bench_hard(algorithm, gen, time_limit);
            }
        }
    }

}

