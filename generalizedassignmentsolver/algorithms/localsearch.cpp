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

class LocalScheme
{

public:

    /*
     * Constructors and destructor
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

    /** Global cost: <Overweight, Cost>; */
    using GlobalCost = std::tuple<Weight, Cost>;

    inline Weight&       overweight(GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Cost&               cost(GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Weight  overweight(const GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Cost          cost(const GlobalCost& global_cost) const { return std::get<1>(global_cost); }

    /*
     * Solutions
     */

    struct Solution
    {
        std::vector<AgentIdx> agents;
        std::vector<Weight> weights;
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
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            if (solution.agents[item_id] != -1)
                continue;
            AgentIdx agent_id = d(generator);
            add(solution, item_id, agent_id);
        }
        return solution;
    }

    inline Solution solution(
            const generalizedassignmentsolver::Solution& solution,
            std::mt19937_64& generator) const
    {
        Solution solution_new = empty_solution();

        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            AgentIdx agent_id = solution.agent(item_id);
            if (agent_id == -1)
                continue;
            add(solution_new, item_id, agent_id);
        }

        std::uniform_int_distribution<ItemIdx> d(0, instance_.number_of_agents() - 1);
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            if (solution_new.agents[item_id] != -1)
                continue;
            AgentIdx agent_id = d(generator);
            add(solution_new, item_id, agent_id);
        }

        return solution_new;
    }

    inline GlobalCost global_cost(const Solution& solution) const
    {
        return {
            solution.overweight,
            solution.cost,
        };
    }

    /*
     * Local search.
     */

    struct MoveShift
    {
        ItemIdx item_id = -1;
        AgentIdx agent_id_old = -1;
        AgentIdx agent_id = -1;
        GlobalCost cost_difference = worst<GlobalCost>();
    };

    struct Perturbation;

    inline void local_search(
            Solution& solution,
            std::mt19937_64&,
            const Perturbation& perturbation = Perturbation())
    {
        // Strucutres for the shift neighborhood.
        // Agents which have changed since the last shift neighborhood
        // exploration.
        optimizationtools::IndexedSet shift_changed_agents(instance_.number_of_agents());
        // Vector containing the improving moves of the shift neighborhood.
        std::vector<MoveShift> shift_improving_moves;

        // Initialize move structures.
        // If we call the local_search on a solution which has not been
        // perturbed, it is because it is not locally optimal. Therefore, all
        // agents are considered 'changed'.
        // Otherwise, only the agent modified by the perturbation are
        // considered 'changed'.
        if (perturbation.moves.empty()) {
            for (AgentIdx agent_id = 0;
                    agent_id < instance_.number_of_agents();
                    ++agent_id) {
                shift_changed_agents.add(agent_id);
            }
        } else {
            for (auto t: perturbation.moves) {
                AgentIdx agent_id_old = std::get<1>(t);
                AgentIdx agent_id = std::get<2>(t);
                shift_changed_agents.add(agent_id);
                shift_changed_agents.add(agent_id_old);
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
                if (shift_changed_agents.contains(it->agent_id_old)
                        || shift_changed_agents.contains(it->agent_id)) {
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
            for (ItemIdx item_id = 0;
                    item_id < instance_.number_of_items();
                    ++item_id) {
                AgentIdx agent_id_old = solution.agents[item_id];
                remove(solution, item_id);
                if (shift_changed_agents.contains(agent_id_old)) {
                    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
                        if (agent_id == agent_id_old)
                            continue;
                        GlobalCost c = cost_add(solution, item_id, agent_id);
                        if (c >= c_cur)
                            continue;
                        MoveShift move;
                        move.item_id = item_id;
                        move.agent_id_old = agent_id_old;
                        move.agent_id = agent_id;
                        move.cost_difference = c - c_cur;
                        shift_improving_moves.push_back(move);
                    }
                } else {
                    for (AgentIdx agent_id: shift_changed_agents) {
                        if (agent_id == agent_id_old)
                            continue;
                        GlobalCost c = cost_add(solution, item_id, agent_id);
                        if (c >= c_cur)
                            continue;
                        MoveShift move;
                        move.item_id = item_id;
                        move.agent_id_old = agent_id_old;
                        move.agent_id = agent_id;
                        move.cost_difference = c - c_cur;
                        shift_improving_moves.push_back(move);
                    }
                }
                add(solution, item_id, agent_id_old);
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
            remove(solution, it_best->item_id);
            add(solution, it_best->item_id, it_best->agent_id);
            if (global_cost(solution) != c_cur + it_best->cost_difference) {
                throw std::logic_error("Costs do not match:\n"
                        "* Current cost: " + to_string(c_cur) + "\n"
                        + "* Move cost difference: " + to_string(it_best->cost_difference) + "\n"
                        + "* Expected new cost: " + to_string(c_cur + it_best->cost_difference) + "\n"
                        + "* Actual new cost: " + to_string(global_cost(solution)) + "\n");
            }

            // Update move structures.
            shift_changed_agents.add(it_best->agent_id_old);
            shift_changed_agents.add(it_best->agent_id);
        }
    }

    /*
     * Iterated local search
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
        for (Counter perturbation_id = 0;
                perturbation_id < parameters_.number_of_perturbations;
                ++perturbation_id) {
            std::vector<ItemIdx> items = optimizationtools::bob_floyd<ItemIdx>(
                    (ItemIdx)8, instance_.number_of_items(), generator);
            std::shuffle(items.begin(), items.end(), generator);
            Perturbation perturbation;
            for (ItemIdx item_id: items) {
                AgentIdx agent_id_old = solution.agents[item_id];
                AgentIdx agent_id_best = -1;
                GlobalCost c_best = worst<GlobalCost>();
                remove(solution, item_id);
                std::shuffle(agents_.begin(), agents_.end(), generator);
                for (AgentIdx agent_id: agents_) {
                    if (agent_id == agent_id_old)
                        continue;
                    GlobalCost c = cost_add(solution, item_id, agent_id);
                    if (agent_id_best == -1 || c < c_best) {
                        agent_id_best = agent_id;
                        c_best = c;
                    }
                }
                add(solution, item_id, agent_id_old);
                perturbation.moves.push_back({item_id, agent_id_old, agent_id_best});
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
            ItemIdx item_id = std::get<0>(t);
            AgentIdx agent_id = std::get<2>(t);
            remove(solution, item_id);
            add(solution, item_id, agent_id);
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
            for (ItemIdx item_id: *compact_solution)
                optimizationtools::hash_combine(hash, hasher(item_id));
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
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            AgentIdx agent_id = compact_solution[item_id];
            if (agent_id != -1)
                add(solution, item_id, agent_id);
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
     * Outputs
     */

    std::ostream& print(
            std::ostream &os,
            const Solution& solution) const
    {
        os << "agents:";
        for (AgentIdx agent_id: solution.agents)
            os << " " << agent_id;
        os << std::endl;
        os << "weights:";
        for (Weight weight: solution.weights)
            os << " " << weight;
        os << std::endl;
        os << "overweight: " << solution.overweight << std::endl;
        os << "cost: " << solution.cost << std::endl;
        return os;
    }

    inline void write(const Solution&, std::string) const { return; }

private:

    /*
     * Manipulate solutions
     */

    inline void add(
            Solution& solution,
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        assert(item_id >= 0);
        assert(item_id < instance_.number_of_items());
        assert(agent_id >= 0);
        assert(agent_id < instance_.number_of_agents());
        assert(solution.agents[item_id] == -1);
        // Update weights.
        Weight w_max = instance_.capacity(agent_id);
        Weight w = instance_.weight(item_id, agent_id);
        if (solution.weights[agent_id] >= w_max) {
            solution.overweight += w;
        } else if (solution.weights[agent_id] + w <= w_max) {
        } else {
            solution.overweight += (solution.weights[agent_id] + w - w_max);
        }
        solution.weights[agent_id] += w;
        // Update cost.
        solution.cost += instance_.cost(item_id, agent_id);
        // Update items.
        solution.agents[item_id] = agent_id;
    }

    inline void remove(
            Solution& solution,
            ItemIdx item_id) const
    {
        AgentIdx agent_id = solution.agents[item_id];
        assert(agent_id != -1);
        // Update weights.
        Weight w_max = instance_.capacity(agent_id);
        Weight w = instance_.weight(item_id, agent_id);
        if (solution.weights[agent_id] - w >= w_max) {
            solution.overweight -= w;
        } else if (solution.weights[agent_id] <= w_max) {
        } else {
            solution.overweight -= (solution.weights[agent_id] - w_max);
        }
        solution.weights[agent_id] -= w;
        // Update cost.
        solution.cost -= instance_.cost(item_id, agent_id);
        // Update items.
        solution.agents[item_id] = -1;
    }

    /*
     * Evaluate moves
     */

    inline GlobalCost cost_add(
            const Solution& solution,
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        GlobalCost gc = global_cost(solution);
        assert(solution.agents[item_id] == -1);
        // Update overweigt.
        Weight w_max = instance_.capacity(agent_id);
        Weight w = instance_.weight(item_id, agent_id);
        if (solution.weights[agent_id] >= w_max) {
            overweight(gc) += w;
        } else if (solution.weights[agent_id] + w <= w_max) {
        } else {
            overweight(gc) += (solution.weights[agent_id] + w - w_max);
        }
        // Update cost.
        cost(gc) += instance_.cost(item_id, agent_id);
        return gc;
    }

    /*
     * Private attributes
     */

    /** Instance. */
    const Instance& instance_;

    /** Parameters. */
    Parameters parameters_;

    std::vector<AgentIdx> agents_;

};

Output generalizedassignmentsolver::localsearch(
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

    Output output(instance, parameters.info);

    // Create LocalScheme.
    LocalScheme::Parameters parameters_local_scheme;
    LocalScheme local_scheme(instance, parameters_local_scheme);

    // Run best first local search.
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
            for (ItemIdx item_id = 0;
                    item_id < instance.number_of_items();
                    ++item_id) {
                AgentIdx agent_id = solution.agents[item_id];
                sol.set(item_id, agent_id);
            }
            std::stringstream ss;
            output.update_solution(sol, ss, parameters.info);
        };
    best_first_local_search(local_scheme, parameters_bfls);

    return output.algorithm_end(parameters.info);
}


