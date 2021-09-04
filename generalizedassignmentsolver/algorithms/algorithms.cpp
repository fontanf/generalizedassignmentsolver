#include "generalizedassignmentsolver/algorithms/algorithms.hpp"

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_volume.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_lbfgs.hpp"
#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"
#include "generalizedassignmentsolver/algorithms/milp_gurobi.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_gecode.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_cplex.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"
#include "generalizedassignmentsolver/algorithms/repair.hpp"
#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/localsolver.hpp"

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
    } catch (const po::required_option& e) {
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
        ("linear-programming-solver,s", po::value<std::string>(&parameters.linear_programming_solver), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
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
        ("threads,t", po::value<Counter>(&parameters.number_of_threads), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
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
    } catch (const po::required_option& e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    return parameters;
}

MilpCplexOptionalParameters read_milp_cplex_args(const std::vector<char*>& argv)
{
    MilpCplexOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("only-linear-relaxation", "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    if (vm.count("only-linear-relaxation"))
        parameters.only_linear_relaxation = true;
    return parameters;
}

#if GUROBI_FOUND
MilpGurobiOptionalParameters read_milp_gurobi_args(const std::vector<char*>& argv)
{
    MilpGurobiOptionalParameters parameters;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("only-linear-relaxation", "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line((Counter)argv.size(), argv.data(), desc), vm);
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
        std::cout << desc << std::endl;;
        throw "";
    }
    if (vm.count("only-linear-relaxation"))
        parameters.only_linear_relaxation = true;
    return parameters;
}
#endif

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
        throw std::invalid_argument("Missing algorithm.");

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
    } else if (algorithm_args[0] == "lagrelax_knapsack_lbfgs") {
        return lagrelax_knapsack_lbfgs(instance, info);
#if COINOR_FOUND
    } else if (algorithm_args[0] == "lagrelax_assignment_volume") {
        return lagrelax_assignment_volume(instance, info);
#endif
    } else if (algorithm_args[0] == "lagrelax_assignment_lbfgs") {
        LagRelaxAssignmentLbfgsOptionalParameters parameters;
        parameters.info = info;
        return lagrelax_assignment_lbfgs(instance, parameters);
    } else if (algorithm_args[0] == "columngeneration") {
        auto parameters = read_columngeneration_args(algorithm_argv);
        parameters.info = info;
        return columngeneration(instance, parameters);

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (algorithm_args[0] == "milp_cbc") {
        MilpCbcOptionalParameters parameters;
        parameters.info = info;
        return milp_cbc(instance, parameters);
#endif
#if CPLEX_FOUND
    } else if (algorithm_args[0] == "milp_cplex") {
        auto parameters = read_milp_cplex_args(algorithm_argv);
        parameters.info = info;
        return milp_cplex(instance, parameters);
#endif
#if GUROBI_FOUND
    } else if (algorithm_args[0] == "milp_gurobi") {
        auto parameters = read_milp_gurobi_args(algorithm_argv);
        parameters.info = info;
        return milp_gurobi(instance, parameters);
#endif
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
        auto parameters = read_repair_args(algorithm_argv);
        parameters.info = info;
        return repair(instance, generator, parameters);
    } else if (algorithm_args[0] == "localsearch") {
        auto parameters = read_localsearch_args(algorithm_argv);
        parameters.info = info;
        parameters.initial_solution = &initial_solution;
        return localsearch(instance, generator, parameters);
#if LOCALocalSearchOLVER_FOUND
    } else if (algorithm_args[0] == "localsolver") {
        LocalSolverOptionalParameters parameters;
        parameters.info = info;
        return localsolver(instance, parameters);
#endif
    } else if (algorithm_args[0] == "columngenerationheuristic_greedy") {
        auto parameters = read_columngeneration_args(algorithm_argv);
        parameters.info = info;
        return columngenerationheuristic_greedy(instance, parameters);
    } else if (algorithm_args[0] == "columngenerationheuristic_limiteddiscrepancysearch") {
        auto parameters = read_columngeneration_args(algorithm_argv);
        parameters.info = info;
        return columngenerationheuristic_limiteddiscrepancysearch(instance, parameters);

    } else {
        throw std::invalid_argument(
                "Unknown algorithm \"" + algorithm_args[0] + "\".");
    }

}

