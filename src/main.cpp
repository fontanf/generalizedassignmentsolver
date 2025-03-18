#include "generalizedassignmentsolver/instance_builder.hpp"

#if CBC_FOUND
//#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#endif
#include "generalizedassignmentsolver/algorithms/column_generation.hpp"
#if CBC_FOUND
#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"
#endif
#if CPLEX_FOUND
#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"
#endif
#if GUROBI_FOUND
#include "generalizedassignmentsolver/algorithms/milp_gurobi.hpp"
#endif
#if KNITRO_FOUND
#include "generalizedassignmentsolver/algorithms/milp_knitro.hpp"
#endif
#if GECODE_FOUND
#include "generalizedassignmentsolver/algorithms/constraint_programming_gecode.hpp"
#endif
#if CPLEX_FOUND
#include "generalizedassignmentsolver/algorithms/constraint_programming_cplex.hpp"
#endif
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"
#include "generalizedassignmentsolver/algorithms/local_search.hpp"
#if LOCALSOLVER_FOUND
#include "generalizedassignmentsolver/algorithms/localsolver.hpp"
#endif

#include <boost/program_options.hpp>

using namespace generalizedassignmentsolver;

namespace po = boost::program_options;

void read_args(
        Parameters& parameters,
        const po::variables_map& vm)
{
    parameters.timer.set_sigint_handler();
    parameters.messages_to_stdout = true;
    if (vm.count("time-limit"))
        parameters.timer.set_time_limit(vm["time-limit"].as<double>());
    if (vm.count("verbosity-level"))
        parameters.verbosity_level = vm["verbosity-level"].as<int>();
    if (vm.count("log"))
        parameters.log_path = vm["log"].as<std::string>();
    parameters.log_to_stderr = vm.count("log-to-stderr");
    bool only_write_at_the_end = vm.count("only-write-at-the-end");
    if (!only_write_at_the_end) {
        std::string certificate_path = vm["certificate"].as<std::string>();
        std::string json_output_path = vm["output"].as<std::string>();
        parameters.new_solution_callback = [
            json_output_path,
            certificate_path](
                    const Output& output,
                    const std::string&)
        {
            output.write_json_output(json_output_path);
            output.solution.write(certificate_path);
        };
    }
}

Output run(
        const Instance& instance,
        const po::variables_map& vm)
{
    std::mt19937_64 generator(vm["seed"].as<Seed>());
    Solution initial_solution(instance, vm["initial-solution"].as<std::string>());

    // Run algorithm.
    std::string algorithm = vm["algorithm"].as<std::string>();
    if (algorithm == "greedy") {
        GreedyParameters parameters;
        read_args(parameters, vm);
        if (vm.count("desirability"))
            parameters.desirability = vm["desirability"].as<std::string>();
        return greedy(instance, parameters);
    } else if (algorithm == "greedy-regret") {
        GreedyParameters parameters;
        read_args(parameters, vm);
        if (vm.count("desirability"))
            parameters.desirability = vm["desirability"].as<std::string>();
        return greedy_regret(instance, parameters);
    } else if (algorithm == "mthg") {
        GreedyParameters parameters;
        read_args(parameters, vm);
        if (vm.count("desirability"))
            parameters.desirability = vm["desirability"].as<std::string>();
        return mthg(instance, parameters);
    } else if (algorithm == "mthg-regret") {
        GreedyParameters parameters;
        read_args(parameters, vm);
        if (vm.count("desirability"))
            parameters.desirability = vm["desirability"].as<std::string>();
        return mthg_regret(instance, parameters);

    } else if (algorithm == "random") {
        Parameters parameters;
        read_args(parameters, vm);
        return random(instance, generator, parameters);

#if CBC_FOUND
    //} else if (algorithm == "linrelax-clp") {
    //    Parameters parameters;
    //    read_args(parameters, vm);
    //    return linrelax_clp(instance, parameters);
#endif
#if CBC_FOUND
    } else if (algorithm == "milp-cbc") {
        MilpCbcParameters parameters;
        if (vm.count("maximum-number-of-nodes"))
            parameters.maximum_number_of_nodes = vm["maximum-number-of-nodes"].as<Counter>();
        read_args(parameters, vm);
        return milp_cbc(instance, parameters);
#endif
#if CPLEX_FOUND
    } else if (algorithm == "milp-cplex") {
        MilpCplexParameters parameters;
        read_args(parameters, vm);
        return milp_cplex(instance, parameters);
#endif
#if GUROBI_FOUND
    } else if (algorithm == "milp-gurobi") {
        MilpGurobiParameters parameters;
        read_args(parameters, vm);
        return milp_gurobi(instance, parameters);
#endif
#if KNITRO_FOUND
    } else if (algorithm == "milp-knitro") {
        MilpKnitroParameters parameters;
        read_args(parameters, vm);
        return milp_knitro(instance, parameters);
#endif

    } else if (algorithm == "column-generation") {
        ColumnGenerationParameters parameters;
        read_args(parameters, vm);
        if (vm.count("linear-programming-solver"))
            parameters.linear_programming_solver = vm["linear-programming-solver"].as<std::string>();
        return column_generation(instance, parameters);
    } else if (algorithm == "column-generation-heuristic-greedy") {
        ColumnGenerationParameters parameters;
        read_args(parameters, vm);
        if (vm.count("linear-programming-solver"))
            parameters.linear_programming_solver = vm["linear-programming-solver"].as<std::string>();
        return column_generation_heuristic_greedy(instance, parameters);
    } else if (algorithm == "column-generation-heuristic-limited-discrepancy-search") {
        ColumnGenerationParameters parameters;
        read_args(parameters, vm);
        if (vm.count("linear-programming-solver"))
            parameters.linear_programming_solver = vm["linear-programming-solver"].as<std::string>();
        return column_generation_heuristic_limited_discrepancy_search(instance, parameters);

#if GECODE_FOUND
    } else if (algorithm == "constraint-programming-gecode") {
        ConstraintProgrammingGecodeParameters parameters;
        read_args(parameters, vm);
        return constraintprogramming_gecode(instance, parameters);
#endif
#if CPLEX_FOUND
    } else if (algorithm == "constraint-programming-cplex") {
        ConstraintProgrammingCplexParameters parameters;
        read_args(parameters, vm);
        return constraintprogramming_cplex(instance, parameters);
#endif

    } else if (algorithm == "local-search") {
        LocalSearchParameters parameters;
        read_args(parameters, vm);
        if (vm.count("maximum-number-of-nodes"))
            parameters.maximum_number_of_nodes = vm["maximum-number-of-nodes"].as<Counter>();
        parameters.initial_solution = &initial_solution;
        return local_search(instance, generator, parameters);
#if LOCALSOLVER_FOUND
    } else if (algorithm == "localsolver") {
        LocalSolverParameters parameters;
        read_args(parameters, vm);
        return localsolver(instance, parameters);
#endif

    } else {
        throw std::invalid_argument(
                "Unknown algorithm \"" + algorithm + "\".");
    }
}

int main(int argc, char *argv[])
{
    // Parse program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::string>()->default_value("large-neighborhood-search"), "set algorithm")
        ("input,i", po::value<std::string>()->required(), "set input file (required)")
        ("format,f", po::value<std::string>()->default_value(""), "set input file format (default: standard)")
        ("unicost,u", "set unicost")
        ("output,o", po::value<std::string>()->default_value(""), "set JSON output file")
        ("initial-solution,", po::value<std::string>()->default_value(""), "")
        ("certificate,c", po::value<std::string>()->default_value(""), "set certificate file")
        ("seed,s", po::value<Seed>()->default_value(0), "set seed")
        ("time-limit,t", po::value<double>(), "set time limit in seconds")
        ("verbosity-level,v", po::value<int>(), "set verbosity level")
        ("only-write-at-the-end,e", "only write output and certificate files at the end")
        ("log,l", po::value<std::string>(), "set log file")
        ("log-to-stderr", "write log to stderr")

        ("desirability,", po::value<std::string>(), "set desirability")
        ("maximum-number-of-nodes,", po::value<Counter>(), "set maximum number of nodes")
        ("linear-programming-solver,", po::value<std::string>(), "set linear programming solver")
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

    // Build instance.
    InstanceBuilder instance_builder;
    instance_builder.read(
            vm["input"].as<std::string>(),
            vm["format"].as<std::string>());
    const Instance instance = instance_builder.build();

    // Run.
    Output output = run(instance, vm);

    // Write outputs.
    std::string certificate_path = vm["certificate"].as<std::string>();
    std::string json_output_path = vm["output"].as<std::string>();
    output.write_json_output(json_output_path);
    output.solution.write(certificate_path);

    return 0;
}
