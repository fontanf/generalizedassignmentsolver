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

ColGenOptionalParameters read_columngeneration_args(const std::vector<char*>& argv)
{
    ColGenOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("solver,s", po::value<std::string>(&parameters.solver), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return parameters;
}

BranchAndPriceOptionalParameters read_branchandprice_args(const std::vector<char*>& argv)
{
    BranchAndPriceOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("solver,s", po::value<std::string>(&parameters.solver), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return parameters;
}

TabuSearchOptionalParameters read_tabusearch_args(const std::vector<char*>& argv)
{
    TabuSearchOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        (",l", po::value<Counter>(&parameters.l), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return parameters;
}

SimulatedAnnealingOptionalParameters read_simulatedannealing_args(const std::vector<char*>& argv)
{
    SimulatedAnnealingOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("beta,b", po::value<double>(&parameters.beta), "")
        (",l", po::value<double>(&parameters.l), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return parameters;
}

Output generalizedassignmentsolver::run(
        std::string algorithm, const Instance& instance, std::mt19937_64& generator, Info info)
{
    std::vector<std::string> algorithm_args = po::split_unix(algorithm);
    std::vector<char*> algorithm_argv;
    for (Counter i = 0; i < (Counter)algorithm_args.size(); ++i)
        algorithm_argv.push_back(const_cast<char*>(algorithm_args[i].c_str()));

    if (algorithm.empty() || algorithm_args[0].empty()) {
        std::cerr << "\033[32m" << "ERROR, missing algorithm." << "\033[0m" << std::endl;
        return Output(instance, info);

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (algorithm_args[0] == "linrelax_clp") {
        return linrelax_clp(instance, info);
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "linrelax_gurobi") {
        return linrelax_gurobi(instance, info);
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_knapsack_volume") {
        return lagrelax_knapsack_volume(instance, info);
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_knapsack_bundle") {
        return lagrelax_knapsack_bundle(instance, info);
#endif
    } else if (algorithm_args[0] == "lagrelax_knapsack_lbfgs") {
        return lagrelax_knapsack_lbfgs(instance, info);
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_assignment_volume") {
        return lagrelax_assignment_volume(instance, info);
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_assignment_bundle") {
        return lagrelax_assignment_bundle(instance, info);
#endif
    } else if (algorithm_args[0] == "lagrelax_assignment_lbfgs") {
        LagRelaxAssignmentLbfgsOptionalParameters parameters;
        parameters.info = info;
        return lagrelax_assignment_lbfgs(instance, parameters);
    } else if (algorithm_args[0] == "columngeneration") {
        ColGenOptionalParameters parameters = read_columngeneration_args(algorithm_argv);
        parameters.info = info;
        return columngeneration(instance, parameters);

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (algorithm_args[0] == "branchandcut_cbc") {
        BranchAndCutCbcOptionalParameters parameters;
        parameters.info = info;
        return branchandcut_cbc(instance, parameters);
#endif
#if CPLEX_FOUND
    } else if (algorithm_args[0] == "branchandcut_cplex") {
        BranchAndCutCplexOptionalParameters parameters;
        parameters.info = info;
        return branchandcut_cplex(instance, parameters);
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "branchandcut_gurobi") {
        BranchAndCutGurobiOptionalParameters parameters;
        parameters.info = info;
        return branchandcut_gurobi(instance, parameters);
#endif
    } else if (algorithm_args[0] == "branchandprice_dfs") {
        BranchAndPriceOptionalParameters parameters = read_branchandprice_args(algorithm_argv);
        parameters.info = info;
        return branchandprice_dfs(instance, parameters);
    } else if (algorithm_args[0] == "branchandprice_astar") {
        BranchAndPriceOptionalParameters parameters = read_branchandprice_args(algorithm_argv);
        parameters.info = info;
        return branchandprice_astar(instance, parameters);
#if GECODE_FOUND
    } else if (algorithm_args[0] == "constraintprogramming_gecode") {
        ConstraintProgrammingGecodeOptionalParameters parameters;
        parameters.info = info;
        return constraintprogramming_gecode(instance, parameters);
#endif
#if CPLEX_FOUND
    } else if (algorithm_args[0] == "constraintprogramming_cplex") {
        ConstraintProgrammingCplexOptionalParameters parameters;
        parameters.info = info;
        return constraintprogramming_cplex(instance, parameters);
#endif

    /*
     * Upper bounds
     */
    } else if (algorithm_args[0] == "random") {
        return random(instance, generator, info);
    } else if (algorithm_args[0] == "greedy") {
        auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
        std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
        std::unique_ptr<Desirability> f = desirability(des_str, instance);
        return greedy(instance, *f, info);
    } else if (algorithm_args[0] == "greedyregret") {
        auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
        std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
        std::unique_ptr<Desirability> f = desirability(des_str, instance);
        return greedyregret(instance, *f, info);
    } else if (algorithm_args[0] == "mthg") {
        auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
        std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
        std::unique_ptr<Desirability> f = desirability(des_str, instance);
        return mthg(instance, *f, info);
    } else if (algorithm_args[0] == "mthgregret") {
        auto it = std::find(algorithm_argv.begin(), algorithm_argv.end(), "f");
        std::string des_str = (it == algorithm_argv.end())? "cij": *(++it);
        std::unique_ptr<Desirability> f = desirability(des_str, instance);
        return mthgregret(instance, *f, info);
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repaircombrelax") {
        return repaircombrelax(instance, info);
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repairgreedy") {
        return repairgreedy(instance, info);
#endif
#if COINOR_FOUND
    } else if (algorithm_args[0] == "repairlinrelax_clp") {
        LinRelaxClpOutput linrelax_output = linrelax_clp(instance);
        return repairlinrelax_clp(instance, linrelax_output, info);
#endif
    } else if (algorithm_args[0] == "localsearch") {
        LocalSearchOptionalParameters parameters;
        parameters.info = info;
        return localsearch(instance, generator, parameters);
    } else if (algorithm_args[0] == "tabusearch") {
        TabuSearchOptionalParameters parameters = read_tabusearch_args(algorithm_argv);
        parameters.info = info;
        return tabusearch(instance, generator, parameters);
    } else if (algorithm_args[0] == "simulatedannealing") {
        SimulatedAnnealingOptionalParameters parameters = read_simulatedannealing_args(algorithm_argv);
        parameters.info = info;
        return simulatedannealing(instance, generator, parameters);
#if LOCALocalSearchOLVER_FOUND
    } else if (algorithm_args[0] == "localsolver") {
        LocalSolverOptionalParameters parameters;
        parameters.info = info;
        return localsolver(instance, parameters);
#endif
    } else if (algorithm_args[0] == "cgh_restrictedmaster") {
        CghRestrictedMasterOptionalParameters parameters;
        parameters.info = info;
        return cgh_restrictedmaster(instance, parameters);
    } else if (algorithm_args[0] == "cgh_purediving") {
        CghPureDivingOptionalParameters parameters;
        parameters.info = info;
        return cgh_purediving(instance, parameters);
    } else if (algorithm_args[0] == "cgh_divingwithlds") {
        CghDivingWithLdsOptionalParameters parameters;
        parameters.info = info;
        return cgh_divingwithlds(instance, parameters);

    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << algorithm_argv[0] << "\033[0m" << std::endl;
        assert(false);
        return Output(instance, info);
    }
}

