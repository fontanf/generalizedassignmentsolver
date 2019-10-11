#include "gap/lib/algorithms.hpp"
#include "gap/lib/generator.hpp"
#include "gap/lib/datasets.hpp"

#include <iomanip>
#include <experimental/filesystem>
#include <boost/program_options.hpp>

using namespace gap;

const std::string forbidden_chars = " \\/:?\"<>|";
static char clear_forbidden(char to_check)
{
    if (forbidden_chars.find(to_check) != std::string::npos)
         return '_';
    return to_check;
}

void bench_normal(
        std::string algorithm,
        std::mt19937_64& gen,
        double time_limit,
        std::string type) // exact, feasible
{
    auto func = get_algorithm(algorithm);

    std::vector<std::pair<ItemIdx, AgentIdx>> nms {
        {100, 10}, {100, 33},
        {1000, 10}, {1000, 100}, {1000, 333},
        {10000, 10}, {10000, 100}, {10000, 1000}, {10000, 3333}};
    std::vector<Weight> rs {100, 1000, 10000};
    std::vector<double> xs {0.0, 0.2, 0.4, 0.6, 0.8, 1};

    nlohmann::json json;
    json["lab"][0] = {"(n,m)", nms};
    json["lab"][1] = {"r", rs};
    json["lab"][2] = {"x", xs};

    int val_max_r = 255 - 52;
    int val_max_g = 255 - 101;
    int val_max_b = 255 - 164;
    int val_max_2_r = 255 - 154;
    int val_max_2_g = 255 - 53;
    int val_max_2_b = 255 - 52;
    int col_r = 0;
    int col_g = 0;
    int col_b = 0;

    Generator data;
    data.t = "n";
    data.s = 0;

    for (Cpt inm = 0; inm < (Cpt)nms.size(); ++inm) {
        ItemIdx n = nms[inm].first;
        ItemIdx m = nms[inm].second;
        data.n = n;
        data.m = m;

        for (Cpt ir = 0; ir < (Cpt)rs.size(); ++ir) {
            Weight r = rs[ir];
            data.r = r;

            // Standard output
            std::stringstream ss;
            ss << "--- n " << n << " m " << m << " r " << r << " --- ";
            int pos = (int)((80 - ss.str().length())/2);
            for(int i=0; i<pos; i++)
                std::cout << " ";
            std::cout << ss.str() << std::endl;

            for (Cpt ix = 0; ix < (Cpt)xs.size(); ++ix) {
                double x = xs[ix];
                data.x = x;
                data.s++;

                // Standard output
                std::cout << "x " << std::right << std::setw(4) << x
                    << " s " << std::right << std::setw(4) << data.s
                    << std::flush;

                Instance ins = data.generate();
                Info info = Info()
                    .set_timelimit(time_limit)
                    //.set_verbose(true)
                    ;
                Output output(ins, info);
                try {
                    output = func(ins, gen, info);
                } catch (...) {
                }

                std::stringstream t_str;
                if (output.time >= 0 && output.time <= time_limit) {
                    t_str.precision(4);
                    t_str << output.time;
                } else {
                    t_str << "> " << time_limit;
                }

                if (type == "feasible") {
                    if (output.time >= 0 && output.time <= time_limit) {
                        if (output.solution.feasible()) {
                            col_r = 255 - (int)(val_max_r * cbrt(output.time / time_limit));
                            col_g = 255 - (int)(val_max_g * cbrt(output.time / time_limit));
                            col_b = 255 - (int)(val_max_b * cbrt(output.time / time_limit));
                            std::cout << "\033[32m";
                        } else {
                            col_r = 255 - (int)(val_max_2_r * cbrt(output.time / time_limit));
                            col_g = 255 - (int)(val_max_2_g * cbrt(output.time / time_limit));
                            col_b = 255 - (int)(val_max_2_b * cbrt(output.time / time_limit));
                        }
                    } else {
                        col_r = 0;
                        col_g = 0;
                        col_b = 0;
                    }
                } else {
                    if (output.time >= 0 && output.time <= time_limit && output.optimal()) {
                        col_r = 255 - (int)(val_max_r * cbrt(output.time / time_limit));
                        col_g = 255 - (int)(val_max_g * cbrt(output.time / time_limit));
                        col_b = 255 - (int)(val_max_b * cbrt(output.time / time_limit));
                        std::cout << "\033[32m";
                    } else {
                        col_r = 0;
                        col_g = 0;
                        col_b = 0;
                    }
                }

                // Json
                std::string rgb_str = "rgb("
                    + std::to_string(col_r) + ","
                    + std::to_string(col_g) + ","
                    + std::to_string(col_b) + ")";
                json["tab"][inm][ir][ix][0]["c"] = rgb_str;
                json["tab"][inm][ir][ix][0]["t"] = t_str.str();

                // Standard output
                std::cout << " | UB" << std::right << std::setw(20) << output.ub_str();
                std::cout << " | LB" << std::right << std::setw(20) << output.lb_str();
                std::cout << " | T (s)" << std::right << std::setw(8) << t_str.str();
                std::cout << "\033[0m" << std::endl;
            }
        }
    }

    std::string filename = algorithm + ".json";
    std::transform(filename.begin(), filename.end(), filename.begin(), clear_forbidden);
    std::ofstream o(filename);
    o << std::setw(4) << json << std::endl;
}

void bench_literature(
        std::string algorithm,
        std::mt19937_64& gen,
        double time_limit)
{
    auto func = get_algorithm(algorithm);

    std::string dir = algorithm;
    while(benchtools::replace(dir, " ", "_"));
    if (time_limit != std::numeric_limits<double>::infinity())
        dir += "_" + std::to_string((int)time_limit);
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

            Info info = Info()
                .set_verbose(false)
                .set_timelimit(time_limit)
                .set_outputfile(outputfile)
                .set_onlywriteattheend(true)
                ;

            Output output = func(ins, gen, info);
            info.write_ini(outputfile);

            if (output.optimal())
                std::cout << "\033[32m";
            std::cout << " | UB" << std::right << std::setw(5) << output.ub_str();
            std::cout << " | LB" << std::setw(5) << output.lb_str();
            std::cout << " | T (s)" << std::right << std::setw(8) << output.time;
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
    std::string type = "exact";

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::vector<std::string>>(&algorithms)->multitoken(), "algorithms (bestfitdecreasing, martello...)")
        ("datasets,d", po::value<std::vector<std::string>>(&datasets)->multitoken(), "datasets (normal, hard)")
        ("time-limit,t", po::value<double>(&time_limit), "time limit in seconds")
        ("type", po::value<std::string>(&type), "feasible, exact")
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

    for (std::string algorithm: algorithms) {
        for (std::string dataset: datasets) {
            std::cout << "*** " << algorithm << " / " << dataset << " ***" << std::endl;

            if (dataset == "normal")
                bench_normal(algorithm, gen, time_limit, type);

            if (dataset == "literature")
                bench_literature(algorithm, gen, time_limit);

        }
    }

}

