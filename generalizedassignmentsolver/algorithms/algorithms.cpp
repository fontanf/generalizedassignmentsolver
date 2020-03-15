#include "generalizedassignmentsolver/algorithms/algorithms.hpp"

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#include "generalizedassignmentsolver/algorithms/linrelax_gurobi.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_volume.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_bundle.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_lbfgs.hpp"
#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cbc.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cplex.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_gurobi.hpp"
#include "generalizedassignmentsolver/algorithms/branchandprice.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_gecode.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_cplex.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"
#include "generalizedassignmentsolver/algorithms/repair.hpp"
#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/localsolver.hpp"
#include "generalizedassignmentsolver/algorithms/colgenheuristics.hpp"

#include <boost/program_options.hpp>

using namespace generalizedassignmentsolver;
namespace po = boost::program_options;

ColGenOptionalParameters read_columngeneration_args(std::vector<char*> argv)
{
    ColGenOptionalParameters p;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("solver,s", po::value<std::string>(&p.solver), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return p;
}

BranchAndPriceOptionalParameters read_branchandprice_args(std::vector<char*> argv)
{
    BranchAndPriceOptionalParameters p;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("solver,s", po::value<std::string>(&p.solver), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return p;
}

TabuSearchOptionalParameters read_tabusearch_args(std::vector<char*> argv)
{
    TabuSearchOptionalParameters p;
    po::options_description desc("Allowed options");
    desc.add_options()
        (",l", po::value<Counter>(&p.l), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return p;
}

SimulatedAnnealingOptionalParameters read_simulatedannealing_args(std::vector<char*> argv)
{
    SimulatedAnnealingOptionalParameters p;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("beta,b", po::value<double>(&p.beta), "")
        (",l", po::value<double>(&p.l), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return p;
}

std::function<Output (Instance&, std::mt19937_64&, Info)> generalizedassignmentsolver::get_algorithm(std::string algorithm)
{
    std::vector<std::string> algorithm_args = po::split_unix(algorithm);
    std::vector<char*> algorithm_argv;
    for(Counter i = 0; i < (Counter)algorithm_args.size(); ++i)
        algorithm_argv.push_back(const_cast<char*>(algorithm_args[i].c_str()));

    if (algorithm_args[0] == "") {
        std::cerr << "\033[32m" << "ERROR, missing algorithm." << "\033[0m" << std::endl;
        return [](Instance& ins, std::mt19937_64&, Info info) { return Output(ins, info); };

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (algorithm_args[0] == "linrelax_clp") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return linrelax_clp(ins, info);
        };
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "linrelax_gurobi") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return linrelax_gurobi(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_knapsack_volume") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lagrelax_knapsack_volume(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_knapsack_bundle") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lagrelax_knapsack_bundle(ins, info);
        };
#endif
    } else if (algorithm_args[0] == "lagrelax_knapsack_lbfgs") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lagrelax_knapsack_lbfgs(ins, info);
        };
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_assignment_volume") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lagrelax_assignment_volume(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_assignment_bundle") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lagrelax_assignment_bundle(ins, info);
        };
#endif
    } else if (algorithm_args[0] == "lagrelax_assignment_lbfgs") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LagRelaxAssignmentLbfgsOptionalParameters p;
            p.info = info;
            return lagrelax_assignment_lbfgs(ins, p);
        };
    } else if (algorithm_args[0] == "columngeneration") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            ColGenOptionalParameters p = read_columngeneration_args(algorithm_argv);
            p.info = info;
            return columngeneration(ins, p);
        };

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (algorithm_args[0] == "branchandcut_cbc") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutCbcOptionalParameters p;
            p.info = info;
            return branchandcut_cbc(ins, p);
        };
#endif
#if CPLEX_FOUND
    } else if (algorithm_args[0] == "branchandcut_cplex") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutCplexOptionalParameters p;
            p.info = info;
            return branchandcut_cplex(ins, p);
        };
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "branchandcut_gurobi") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutGurobiOptionalParameters p;
            p.info = info;
            return branchandcut_gurobi(ins, p);
        };
#endif
    } else if (algorithm_args[0] == "branchandprice_dfs") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndPriceOptionalParameters p = read_branchandprice_args(algorithm_argv);
            p.info = info;
            return branchandprice_dfs(ins, p);
        };
    } else if (algorithm_args[0] == "branchandprice_astar") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndPriceOptionalParameters p = read_branchandprice_args(algorithm_argv);
            p.info = info;
            return branchandprice_astar(ins, p);
        };
#if GECODE_FOUND
    } else if (algorithm_args[0] == "constraintprogramming_gecode") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            ConstraintProgrammingGecodeOptionalParameters p;
            p.info = info;
            return constraintprogramming_gecode(ins, p);
        };
#endif
#if CPLEX_FOUND
    } else if (algorithm_args[0] == "constraintprogramming_cplex") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            ConstraintProgrammingCplexOptionalParameters p;
            p.info = info;
            return constraintprogramming_cplex(ins, p);
        };
#endif

    /*
     * Upper bounds
     */
    } else if (algorithm_args[0] == "random") {
        return [](Instance& ins, std::mt19937_64& gen, Info info) {
            return random(ins, gen, info);
        };
    } else if (algorithm_args[0] == "greedy") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
            std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return greedy(ins, *f, info);
        };
    } else if (algorithm_args[0] == "greedyregret") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
            std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return greedyregret(ins, *f, info);
        };
    } else if (algorithm_args[0] == "mthg") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
            std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return mthg(ins, *f, info);
        };
    } else if (algorithm_args[0] == "mthgregret") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
            std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return mthgregret(ins, *f, info);
        };
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repaircombrelax") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return repaircombrelax(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repairgreedy") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return repairgreedy(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repairlinrelax_clp") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LinRelaxClpOutput linrelax_output = linrelax_clp(ins);
            return repairlinrelax_clp(ins, linrelax_output, info);
        };
#endif
    } else if (algorithm_args[0] == "localsearch") {
        return [algorithm_argv](Instance& ins, std::mt19937_64& gen, Info info) {
            LocalSearchOptionalParameters p;
            p.info = info;
            return localsearch(ins, gen, p);
        };
    } else if (algorithm_args[0] == "tabusearch") {
        return [algorithm_argv](Instance& ins, std::mt19937_64& gen, Info info) {
            TabuSearchOptionalParameters p = read_tabusearch_args(algorithm_argv);
            p.info = info;
            return tabusearch(ins, gen, p);
        };
    } else if (algorithm_args[0] == "simulatedannealing") {
        return [algorithm_argv](Instance& ins, std::mt19937_64& gen, Info info) {
            SimulatedAnnealingOptionalParameters p = read_simulatedannealing_args(algorithm_argv);
            p.info = info;
            return simulatedannealing(ins, gen, p);
        };
#if LOCALocalSearchOLVER_FOUND
    } else if (algorithm_args[0] == "localsolver") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LocalSolverOptionalParameters p;
            p.info = info;
            return localsolver(ins, p);
        };
#endif
    } else if (algorithm_args[0] == "cgh_restrictedmaster") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            CghRestrictedMasterOptionalParameters p;
            p.info = info;
            return cgh_restrictedmaster(ins, p);
        };
    } else if (algorithm_args[0] == "cgh_purediving") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            CghPureDivingOptionalParameters p;
            p.info = info;
            return cgh_purediving(ins, p);
        };
    } else if (algorithm_args[0] == "cgh_divingwithlds") {
        return [algorithm_argv](Instance& ins, std::mt19937_64&, Info info) {
            CghDivingWithLdsOptionalParameters p;
            p.info = info;
            return cgh_divingwithlds(ins, p);
        };

    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << algorithm_argv[0] << "\033[0m" << std::endl;
        assert(false);
        return [](Instance& ins, std::mt19937_64&, Info info) { return Output(ins, info); };
    }
}

