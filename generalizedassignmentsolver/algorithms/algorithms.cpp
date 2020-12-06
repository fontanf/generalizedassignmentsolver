#include "generalizedassignmentsolver/algorithms/algorithms.hpp"

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
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

std::string read_desiralibity_args(const std::vector<char*>& argv)
{
    ColumnGenerationOptionalParameters parameters;
    po::options_description desc("Allowed options");
    std::string desirability = "cij";
    desc.add_options()
        (",f", po::value<std::string>(&desirability), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return desirability;
}

ColumnGenerationOptionalParameters read_columngeneration_args(const std::vector<char*>& argv)
{
    ColumnGenerationOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("lp-solver,s", po::value<std::string>(&parameters.lp_solver), "")
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

CghGreedyOptionalParameters read_cgh_greedy_args(const std::vector<char*>& argv)
{
    CghGreedyOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("lp-solver,s", po::value<std::string>(&parameters.lp_solver), "")
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

CghLimitedDiscrepencySearchOptionalParameters read_cgh_limiteddiscrepencysearch_args(const std::vector<char*>& argv)
{
    CghLimitedDiscrepencySearchOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("lp-solver,s", po::value<std::string>(&parameters.lp_solver), "")
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
        ("lp-solver,s", po::value<std::string>(&parameters.lp_solver), "")
        ("tree-search-algorithm,t", po::value<std::string>(&parameters.tree_search_algorithm), "")
        ("branching-rule,b", po::value<std::string>(&parameters.branching_rule), "")
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

LocalSearchOptionalParameters read_localsearch_args(const std::vector<char*>& argv)
{
    LocalSearchOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("iteration-limit,i", po::value<Counter>(&parameters.iteration_limit), "")
        ("iteration-without-improvment-limit,w", po::value<Counter>(&parameters.iteration_without_improvment_limit), "")
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
        ("iteration-limit,i", po::value<Counter>(&parameters.iteration_limit), "")
        ("iteration-without-improvment-limit,w", po::value<Counter>(&parameters.iteration_without_improvment_limit), "")
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
        ("iteration-limit,i", po::value<Counter>(&parameters.iteration_limit), "")
        ("iteration-without-improvment-limit,w", po::value<Counter>(&parameters.iteration_without_improvment_limit), "")
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

RepairOptionalParameters read_repair_args(const std::vector<char*>& argv)
{
    RepairOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("initial-solution,i", po::value<RepairInitialSolution>(&parameters.initial_solution), "")
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

BranchAndCutCplexOptionalParameters read_branchandcut_cplex_args(const std::vector<char*>& argv)
{
    BranchAndCutCplexOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("only-linear-relaxation", "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    if (vm.count("only-linear-relaxation"))
        parameters.only_linear_relaxation = true;
    return parameters;
}

BranchAndCutGurobiOptionalParameters read_branchandcut_gurobi_args(const std::vector<char*>& argv)
{
    BranchAndCutGurobiOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("only-linear-relaxation", "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (po::required_option e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    if (vm.count("only-linear-relaxation"))
        parameters.only_linear_relaxation = true;
    return parameters;
}

Output generalizedassignmentsolver::run(
        std::string algorithm,
        const Instance& instance,
        const Solution& initial_solution,
        std::mt19937_64& generator,
        Info info)
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
        ColumnGenerationOptionalParameters parameters = read_columngeneration_args(algorithm_argv);
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
        auto parameters = read_branchandcut_cplex_args(algorithm_argv);
        parameters.info = info;
        return branchandcut_cplex(instance, parameters);
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "branchandcut_gurobi") {
        auto parameters = read_branchandcut_gurobi_args(algorithm_argv);
        parameters.info = info;
        return branchandcut_gurobi(instance, parameters);
#endif
    } else if (algorithm_args[0] == "branchandprice") {
        auto parameters = read_branchandprice_args(algorithm_argv);
        parameters.info = info;
        return branchandprice(instance, parameters);
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
        std::string desirability_string = read_desiralibity_args(algorithm_argv);
        std::unique_ptr<Desirability> f = desirability(desirability_string, instance);
        return greedy(instance, *f, info);
    } else if (algorithm_args[0] == "greedyregret") {
        std::string desirability_string = read_desiralibity_args(algorithm_argv);
        std::unique_ptr<Desirability> f = desirability(desirability_string, instance);
        return greedyregret(instance, *f, info);
    } else if (algorithm_args[0] == "mthg") {
        std::string desirability_string = read_desiralibity_args(algorithm_argv);
        std::unique_ptr<Desirability> f = desirability(desirability_string, instance);
        return mthg(instance, *f, info);
    } else if (algorithm_args[0] == "mthgregret") {
        std::string desirability_string = read_desiralibity_args(algorithm_argv);
        std::unique_ptr<Desirability> f = desirability(desirability_string, instance);
        return mthgregret(instance, *f, info);
    } else if (algorithm_args[0] == "repair") {
        RepairOptionalParameters parameters = read_repair_args(algorithm_argv);
        parameters.info = info;
        return repair(instance, generator, parameters);
    } else if (algorithm_args[0] == "localsearch") {
        LocalSearchOptionalParameters parameters = read_localsearch_args(algorithm_argv);
        parameters.info = info;
        parameters.initial_solution = &initial_solution;
        return localsearch(instance, generator, parameters);
    } else if (algorithm_args[0] == "tabusearch") {
        TabuSearchOptionalParameters parameters = read_tabusearch_args(algorithm_argv);
        parameters.info = info;
        parameters.initial_solution = &initial_solution;
        return tabusearch(instance, generator, parameters);
    } else if (algorithm_args[0] == "simulatedannealing") {
        SimulatedAnnealingOptionalParameters parameters = read_simulatedannealing_args(algorithm_argv);
        parameters.info = info;
        parameters.initial_solution = &initial_solution;
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
    } else if (algorithm_args[0] == "cgh_greedy") {
        CghGreedyOptionalParameters parameters = read_cgh_greedy_args(algorithm_argv);
        parameters.info = info;
        return cgh_greedy(instance, parameters);
    } else if (algorithm_args[0] == "cgh_limiteddiscrepencysearch") {
        CghLimitedDiscrepencySearchOptionalParameters parameters = read_cgh_limiteddiscrepencysearch_args(algorithm_argv);
        parameters.info = info;
        return cgh_limiteddiscrepencysearch(instance, parameters);

    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << algorithm_argv[0] << "\033[0m" << std::endl;
        assert(false);
        return Output(instance, info);
    }

}

