#include "gap/lib/algorithms.hpp"
#include "gap/lib/datasets.hpp"

#include <iomanip>
#include <experimental/filesystem>

using namespace gap;

bool replace(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
        return 1;
    std::string algorithm = argv[1];
    double time_limit = std::numeric_limits<double>::infinity();
    if (argc > 2)
        time_limit = std::stod(argv[2]);
    int seed = 0;

    std::mt19937_64 gen(seed);
    auto func = get_algorithm(algorithm);

    std::string dir = algorithm;
    replace(dir, " ", "_");
    if (time_limit != std::numeric_limits<double>::infinity())
        dir += "_" + std::to_string(time_limit);
    std::experimental::filesystem::create_directory(dir);

    for (auto it = datasets.begin(); it != datasets.end(); ++it) {
        Dataset<Instance>* d = it->second.get();
        for (InsId i=0; i<d->size(); ++i) {
            Instance ins = d->instance(i);

            std::string s = ins.name();
            replace(s, "data", dir);

            std::string outputfile = s + ".ini";
            std::cout << std::left << std::setw(16) << ins.name() << std::flush;

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

            double ub = (!sol.feasible())? std::numeric_limits<double>::infinity(): sol.cost();
            double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
            std::cout << std::left << std::setw(6) << " | UB";
            std::cout << std::right << std::setw(8) << ub;
            std::cout << std::left << std::setw(6) << " | LB";
            std::cout << std::right << std::setw(8) << lb;
            std::cout << std::left << std::setw(8) << " | T (s)";
            std::cout << std::right << std::setw(10) << t;
            std::cout << std::endl;
        }
    }

}

