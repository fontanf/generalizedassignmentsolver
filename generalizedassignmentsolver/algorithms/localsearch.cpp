#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include "localsearchsolver/best_first_local_search.hpp"

#include "optimizationtools/containers/indexed_set.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;
using namespace localsearchsolver;

LocalSearchOutput& LocalSearchOutput::algorithm_end(
        optimizationtools::Info& info)
{
    //info.add_to_json("Algorithm", "Iterations", iterations);
    Output::algorithm_end(info);
    //FFOT_VER(info, "Iterations: " << iterations << std::endl);
    return *this;
}

class LocalScheme
{

public:

    /*
     * Constructors and destructor.
     */

    struct Parameters
    {
        Counter number_of_perturbations = 10;
    };

    LocalScheme(
            const Instance& instance,
            Parameters parameters):
        instance_(instance),
        parameters_(parameters),
        agents_(instance.number_of_agents())
    {
        std::iota(agents_.begin(), agents_.end(), 0);
    }

    /*
     * Global cost.
     */

    /** Global cost: <Item number, Overweight, Cost>; */
    using GlobalCost = std::tuple<ItemIdx, Weight, Cost>;

    inline ItemIdx&       number_of_items(GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Weight&             overweight(GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Cost&                     cost(GlobalCost& global_cost) const { return std::get<2>(global_cost); }
    inline ItemIdx  number_of_items(const GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Weight        overweight(const GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Cost                cost(const GlobalCost& global_cost) const { return std::get<2>(global_cost); }

    /*
     * Solutions.
     */

    struct Solution
    {
        std::vector<AgentIdx> agents;
        std::vector<Weight> weights;
        ItemIdx number_of_items = 0;
        Weight overweight = 0;
        Cost cost = 0;
    };

    inline Solution empty_solution() const
    {
        Solution solution;
        solution.agents.resize(instance_.number_of_items(), -1);
        solution.weights.resize(instance_.number_of_agents(), 0);
        return solution;
    }

    inline Solution initial_solution(
            Counter,
            std::mt19937_64& generator) const
    {
        Solution solution = empty_solution();
        std::uniform_int_distribution<ItemIdx> d(0, instance_.number_of_agents() - 1);
        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            if (solution.agents[j] != -1)
                continue;
            AgentIdx i = d(generator);
            add(solution, j, i);
        }
        return solution;
    }

    inline Solution solution(
            const generalizedassignmentsolver::Solution& solution,
            std::mt19937_64& generator) const
    {
        Solution solution_new = empty_solution();

        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            AgentIdx i = solution.agent(j);
            if (i == -1)
                continue;
            add(solution_new, j, i);
        }

        std::uniform_int_distribution<ItemIdx> d(0, instance_.number_of_agents() - 1);
        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            if (solution_new.agents[j] != -1)
                continue;
            AgentIdx i = d(generator);
            add(solution_new, j, i);
        }

        return solution_new;
    }

    inline GlobalCost global_cost(const Solution& solution) const
    {
        return {
            -solution.number_of_items,
            solution.overweight,
            solution.cost,
        };
    }

    /*
     * Local search.
     */

    struct MoveShift
    {
        ItemIdx j = -1;
        AgentIdx i_old = -1;
        AgentIdx i = -1;
        GlobalCost cost_difference = worst<GlobalCost>();
    };

    struct Perturbation;

    inline void local_search(
            Solution& solution,
            std::mt19937_64&,
            const Perturbation& perturbation = Perturbation())
    {
        ItemIdx n = instance_.number_of_items();
        AgentIdx m = instance_.number_of_agents();

        // Strucutres for the shift neighborhood.
        // Agents which have changed since the last shift neighborhood
        // exploration.
        optimizationtools::IndexedSet shift_changed_agents(m);
        // Vector containing the improving moves of the shift neighborhood.
        std::vector<MoveShift> shift_improving_moves;

        // Initialize move structures.
        // If we call the local_search on a solution which has not been
        // perturbed, it is because it is not locally optimal. Therefore, all
        // agents are considered 'changed'.
        // Otherwise, only the agent modified by the perturbation are
        // considered 'changed'.
        if (perturbation.moves.empty()) {
            for (AgentIdx i = 0; i < m; ++i)
                shift_changed_agents.add(i);
        } else {
            for (auto t: perturbation.moves) {
                AgentIdx i_old = std::get<1>(t);
                AgentIdx i = std::get<2>(t);
                shift_changed_agents.add(i);
                shift_changed_agents.add(i_old);
            }
        }

        Counter it = 0;
        (void)it;
        for (;; ++it) {
            //std::cout << "it " << it << " cost " << to_string(global_cost(solution)) << std::endl;

            // Shift neighborhood exploration.

            // Remove obsolete moves.
            for (auto it = shift_improving_moves.begin();
                    it != shift_improving_moves.end();) {
                if (shift_changed_agents.contains(it->i_old)
                        || shift_changed_agents.contains(it->i)) {
                    *it = *std::prev(shift_improving_moves.end());
                    shift_improving_moves.pop_back();
                } else {
                    ++it;
                }
            }

            // Evaluate new moves.
            // For each item belonging to a changed agent, we add shift moves
            // toward all agents.
            // For each item belonging to an unchanged agent, we add shift
            // moves toward all changed agents.
            GlobalCost c_cur = global_cost(solution);
            for (ItemIdx j = 0; j < n; ++j) {
                AgentIdx i_old = solution.agents[j];
                remove(solution, j);
                if (shift_changed_agents.contains(i_old)) {
                    for (AgentIdx i = 0; i < m; ++i) {
                        if (i == i_old)
                            continue;
                        GlobalCost c = cost_add(solution, j, i);
                        if (c >= c_cur)
                            continue;
                        MoveShift move;
                        move.j = j;
                        move.i_old = i_old;
                        move.i = i;
                        move.cost_difference = c - c_cur;
                        shift_improving_moves.push_back(move);
                    }
                } else {
                    for (AgentIdx i: shift_changed_agents) {
                        if (i == i_old)
                            continue;
                        GlobalCost c = cost_add(solution, j, i);
                        if (c >= c_cur)
                            continue;
                        MoveShift move;
                        move.j = j;
                        move.i_old = i_old;
                        move.i = i;
                        move.cost_difference = c - c_cur;
                        shift_improving_moves.push_back(move);
                    }
                }
                add(solution, j, i_old);
            }
            shift_changed_agents.clear();

            // If there is no improving move, then stop here.
            if (shift_improving_moves.empty())
                break;

            // Otherwise, look for a pareto improving move.
            //std::shuffle(shift_improving_moves.begin(), shift_improving_moves.end(), generator);
            auto it_best = shift_improving_moves.begin();
            for (auto it = shift_improving_moves.begin() + 1;
                    it != shift_improving_moves.end(); ++it) {
                if (it->cost_difference >= it_best->cost_difference
                        || !dominates(it->cost_difference, it_best->cost_difference))
                    continue;
                it_best = it;
            }

            // Apply move.
            remove(solution, it_best->j);
            add(solution, it_best->j, it_best->i);
            if (global_cost(solution) != c_cur + it_best->cost_difference) {
                throw std::logic_error("Costs do not match:\n"
                        "* Current cost: " + to_string(c_cur) + "\n"
                        + "* Move cost difference: " + to_string(it_best->cost_difference) + "\n"
                        + "* Expected new cost: " + to_string(c_cur + it_best->cost_difference) + "\n"
                        + "* Actual new cost: " + to_string(global_cost(solution)) + "\n");
            }

            // Update move structures.
            shift_changed_agents.add(it_best->i_old);
            shift_changed_agents.add(it_best->i);
        }
    }

    /*
     * Iterated local search.
     */

    struct Perturbation
    {
        std::vector<std::tuple<ItemIdx, AgentIdx, AgentIdx>> moves;
        GlobalCost global_cost;
    };

    inline std::vector<Perturbation> perturbations(
            Solution& solution,
            std::mt19937_64& generator)
    {
        std::vector<Perturbation> perturbations;
        for (Counter perturbation_id = 0; perturbation_id < parameters_.number_of_perturbations; ++perturbation_id) {
            std::vector<ItemIdx> items = optimizationtools::bob_floyd<ItemIdx>(
                    (ItemIdx)8, instance_.number_of_items(), generator);
            std::shuffle(items.begin(), items.end(), generator);
            Perturbation perturbation;
            for (ItemIdx j: items) {
                AgentIdx i_old = solution.agents[j];
                AgentIdx i_best = -1;
                GlobalCost c_best = worst<GlobalCost>();
                remove(solution, j);
                std::shuffle(agents_.begin(), agents_.end(), generator);
                for (AgentIdx i: agents_) {
                    if (i == i_old)
                        continue;
                    GlobalCost c = cost_add(solution, j, i);
                    if (i_best == -1 || c < c_best) {
                        i_best = i;
                        c_best = c;
                    }
                }
                add(solution, j, i_old);
                perturbation.moves.push_back({j, i_old, i_best});
            }
            perturbation.global_cost = global_cost(solution);
            perturbations.push_back(perturbation);
        }
        return perturbations;
    }

    inline void apply_perturbation(
            Solution& solution,
            const Perturbation& perturbation,
            std::mt19937_64&) const
    {
        for (auto t: perturbation.moves) {
            ItemIdx j = std::get<0>(t);
            AgentIdx i = std::get<2>(t);
            remove(solution, j);
            add(solution, j, i);
        }
    }

    /*
     * Best first local search.
     */

    using CompactSolution = std::vector<AgentIdx>;

    struct CompactSolutionHasher
    {
        std::hash<AgentIdx> hasher;

        inline bool operator()(
                const std::shared_ptr<CompactSolution>& compact_solution_1,
                const std::shared_ptr<CompactSolution>& compact_solution_2) const
        {
            return *compact_solution_1 == *compact_solution_2;
        }

        inline std::size_t operator()(
                const std::shared_ptr<CompactSolution>& compact_solution) const
        {
            size_t hash = 0;
            for (ItemIdx j: *compact_solution)
                optimizationtools::hash_combine(hash, hasher(j));
            return hash;
        }
    };

    inline CompactSolutionHasher compact_solution_hasher() const { return CompactSolutionHasher(); }

    CompactSolution solution2compact(const Solution& solution)
    {
        return solution.agents;
    }

    Solution compact2solution(const CompactSolution& compact_solution)
    {
        Solution solution = empty_solution();
        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            AgentIdx i = compact_solution[j];
            if (i != -1)
                add(solution, j, i);
        }
        return solution;
    }

    struct PerturbationHasher
    {
        inline bool hashable(const Perturbation&) const { return false; }
        inline bool operator()(const Perturbation&, const Perturbation&) const { return false; }
        inline std::size_t operator()(const Perturbation&) const { return 0; }
    };

    inline PerturbationHasher perturbation_hasher() const { return PerturbationHasher(); }

    /*
     * Outputs.
     */

    std::ostream& print(
            std::ostream &os,
            const Solution& solution) const
    {
        os << "item number: " << solution.number_of_items << std::endl;
        os << "agents:";
        for (AgentIdx i: solution.agents)
            os << " " << i;
        os << std::endl;
        os << "weights:";
        for (Weight w: solution.weights)
            os << " " << w;
        os << std::endl;
        os << "overweight: " << solution.overweight << std::endl;
        os << "cost: " << solution.cost << std::endl;
        return os;
    }

    inline void write(const Solution&, std::string) const { return; }

private:

    /*
     * Manipulate solutions.
     */

    inline void add(Solution& solution, ItemIdx j, AgentIdx i) const
    {
        assert(j >= 0);
        assert(j < instance_.number_of_items());
        assert(i >= 0);
        assert(i < instance_.number_of_agents());
        assert(solution.agents[j] == -1);
        // Update number_of_items.
        solution.number_of_items++;
        // Update weights.
        Weight w_max = instance_.capacity(i);
        Weight w = instance_.weight(j, i);
        if (solution.weights[i] >= w_max) {
            solution.overweight += w;
        } else if (solution.weights[i] + w <= w_max) {
        } else {
            solution.overweight += (solution.weights[i] + w - w_max);
        }
        solution.weights[i] += w;
        // Update cost.
        solution.cost += instance_.cost(j, i);
        // Update items.
        solution.agents[j] = i;
    }

    inline void remove(Solution& solution, ItemIdx j) const
    {
        AgentIdx i = solution.agents[j];
        assert(i != -1);
        // Update number_of_items.
        solution.number_of_items--;
        // Update weights.
        Weight w_max = instance_.capacity(i);
        Weight w = instance_.weight(j, i);
        if (solution.weights[i] - w >= w_max) {
            solution.overweight -= w;
        } else if (solution.weights[i] <= w_max) {
        } else {
            solution.overweight -= (solution.weights[i] - w_max);
        }
        solution.weights[i] -= w;
        // Update cost.
        solution.cost -= instance_.cost(j, i);
        // Update items.
        solution.agents[j] = -1;
    }

    /*
     * Evaluate moves.
     */

    inline GlobalCost cost_add(
            const Solution& solution,
            ItemIdx j,
            AgentIdx i) const
    {
        GlobalCost gc = global_cost(solution);
        assert(solution.agents[j] == -1);
        // Update number_of_items.
        number_of_items(gc)--;
        // Update overweigt.
        Weight w_max = instance_.capacity(i);
        Weight w = instance_.weight(j, i);
        if (solution.weights[i] >= w_max) {
            overweight(gc) += w;
        } else if (solution.weights[i] + w <= w_max) {
        } else {
            overweight(gc) += (solution.weights[i] + w - w_max);
        }
        // Update cost.
        cost(gc) += instance_.cost(j, i);
        return gc;
    }

    /*
     * Private attributes.
     */

    const Instance& instance_;
    Parameters parameters_;

    std::vector<AgentIdx> agents_;

};

LocalSearchOutput generalizedassignmentsolver::localsearch(
        const Instance& instance,
        std::mt19937_64& generator,
        LocalSearchOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Local Search" << std::endl
            << std::endl;

    LocalSearchOutput output(instance, parameters.info);

    // Create LocalScheme.
    LocalScheme::Parameters parameters_local_scheme;
    LocalScheme local_scheme(instance, parameters_local_scheme);

    // Run A*.
    BestFirstLocalSearchOptionalParameters<LocalScheme> parameters_bfls;
    parameters_bfls.info.set_verbosity_level(0);
    parameters_bfls.info.set_time_limit(parameters.info.remaining_time());
    parameters_bfls.number_of_threads_1 = 1;
    parameters_bfls.number_of_threads_2 = parameters.number_of_threads;
    if (parameters.initial_solution == nullptr) {
        parameters_bfls.initial_solution_ids = std::vector<Counter>(
                parameters_bfls.number_of_threads_2, 0);
    } else {
        LocalScheme::Solution solution = local_scheme.solution(*parameters.initial_solution, generator);
        parameters_bfls.initial_solution_ids = {};
        parameters_bfls.initial_solutions = {solution};
    }
    parameters_bfls.new_solution_callback
        = [&instance, &parameters, &output](
                const LocalScheme::Solution& solution)
        {
            Solution sol(instance);
            for (ItemIdx j = 0; j < instance.number_of_items(); ++j) {
                AgentIdx i = solution.agents[j];
                sol.set(j, i);
            }
            std::stringstream ss;
            output.update_solution(sol, ss, parameters.info);
        };
    best_first_local_search(local_scheme, parameters_bfls);

    return output.algorithm_end(parameters.info);
}


