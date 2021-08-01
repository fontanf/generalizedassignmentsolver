#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include "localsearchsolver/a_star_local_search.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;
using namespace localsearchsolver;

LocalSearchOutput& LocalSearchOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", iterations);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << iterations << std::endl);
    return *this;
}

class LocalScheme
{

public:

    /** Global cost: <Item number, Overweight, Cost>; */
    using GlobalCost = std::tuple<ItemIdx, Weight, Cost>;

    inline ItemIdx&       number_of_items(GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Weight&         overweight(GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Cost&                 cost(GlobalCost& global_cost) const { return std::get<2>(global_cost); }
    inline ItemIdx  number_of_items(const GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Weight    overweight(const GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Cost            cost(const GlobalCost& global_cost) const { return std::get<2>(global_cost); }

    static GlobalCost global_cost_worst()
    {
        return {
            std::numeric_limits<ItemIdx>::max(),
            std::numeric_limits<Weight>::max(),
            std::numeric_limits<Cost>::max(),
        };
    }

    /*
     * Solutions.
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

    struct Solution
    {
        std::vector<AgentIdx> agents;
        std::vector<Weight> weights;
        ItemIdx number_of_items = 0;
        Weight overweight = 0;
        Cost cost = 0;
    };

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

    /*
     * Constructors and destructor.
     */

    struct Parameters
    {
    };

    LocalScheme(
            const Instance& instance,
            Parameters parameters):
        instance_(instance),
        parameters_(parameters),
        items_(instance.number_of_items()),
        agents_(instance.number_of_agents())
    {
        std::iota(items_.begin(), items_.end(), 0);
        std::iota(agents_.begin(), agents_.end(), 0);
    }

    LocalScheme(const LocalScheme& local_scheme):
        LocalScheme(local_scheme.instance_, local_scheme.parameters_) { }

    virtual ~LocalScheme() { }

    /*
     * Initial solutions.
     */

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
            const generalizedassignmentsolver::Solution& solution) const
    {
        Solution solution_new = empty_solution();
        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            AgentIdx i = solution.agent(j);
            add(solution_new, j, i);
        }
        return solution_new;
    }

    /*
     * Solution properties.
     */

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

    struct Move
    {
        ItemIdx j;
        AgentIdx i;
        GlobalCost global_cost;
    };

    static Move move_null() { return {-1, -1, global_cost_worst()}; }

    struct MoveHasher
    {
        std::hash<ItemIdx> hasher_1;
        std::hash<AgentIdx> hasher_2;

        inline bool hashable(const Move&) const { return true; }

        inline bool operator()(
                const Move& move_1,
                const Move& move_2) const
        {
            return move_1.j == move_2.j
                && move_1.i == move_2.i;
        }

        inline std::size_t operator()(
                const Move& move) const
        {
            auto hash = hasher_1(move.j);
            optimizationtools::hash_combine(hash, hasher_2(move.i));
            return hash;
        }
    };

    inline MoveHasher move_hasher() const { return MoveHasher(); }

    inline std::vector<Move> perturbations(
            Solution& solution,
            std::mt19937_64&) const
    {
        std::vector<Move> moves;
        for (ItemIdx j = 0; j < instance_.number_of_items(); ++j) {
            AgentIdx i_old = solution.agents[j];
            remove(solution, j);
            for (AgentIdx i = 0; i < instance_.number_of_agents(); ++i) {
                if (i == i_old)
                    continue;
                Move move;
                move.j = j;
                move.i = i;
                move.global_cost = cost_add(solution, j, i);
                moves.push_back(move);
            }
            add(solution, j, i_old);
        }
        return moves;
    }

    inline void apply_move(Solution& solution, const Move& move) const
    {
        remove(solution, move.j);
        add(solution, move.j, move.i);
    }

    inline void local_search(
            Solution& solution,
            std::mt19937_64& generator,
            const Move& tabu = move_null())
    {
        Counter it = 0;
        for (;; ++it) {
            //std::cout << "it " << it << " cost " << to_string(global_cost(solution)) << std::endl;
            std::shuffle(items_.begin(), items_.end(), generator);
            std::shuffle(agents_.begin(), agents_.end(), generator);
            ItemIdx j_best = -1;
            AgentIdx i_best = -1;
            GlobalCost c_best = global_cost(solution);
            for (auto& j: items_) {
                AgentIdx i_old = solution.agents[j];
                remove(solution, j);
                for (AgentIdx i: agents_) {
                    if (i == i_old)
                        continue;
                    if (j == tabu.j && i == tabu.i)
                        continue;
                    GlobalCost c = cost_add(solution, j, i);
                    if (c >= c_best)
                        continue;
                    if (j_best != -1 && !dominates(c, c_best))
                        continue;
                    j_best = j;
                    i_best = i;
                    c_best = c;
                }
                add(solution, j, i_old);
            }
            if (j_best == -1)
                break;
            remove(solution, j_best);
            add(solution, j_best, i_best);
            if (solution.cost != cost(c_best)) {
                std::cout << to_string(c_best) << std::endl;
                std::cout << to_string(global_cost(solution)) << std::endl;
            }
            assert(solution.cost == cost(c_best));
        }
    }

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

    std::vector<ItemIdx> items_;
    std::vector<AgentIdx> agents_;

};

LocalSearchOutput generalizedassignmentsolver::localsearch(
        const Instance& instance,
        std::mt19937_64&,
        LocalSearchOptionalParameters parameters)
{
    LocalSearchOutput output(instance, parameters.info);

    // Create LocalScheme.
    LocalScheme::Parameters parameters_local_scheme;
    LocalScheme local_scheme(instance, parameters_local_scheme);

    // Run A*.
    AStarLocalSearchOptionalParameters<LocalScheme> parameters_a_star;
    parameters_a_star.info.set_verbose(false);
    parameters_a_star.info.set_time_limit(parameters.info.remaining_time());
    parameters_a_star.number_of_threads_1 = 1;
    parameters_a_star.number_of_threads_2 = parameters.number_of_threads;
    if (parameters.initial_solution == nullptr) {
        parameters_a_star.initial_solution_ids = std::vector<Counter>(
                parameters_a_star.number_of_threads_2, 0);
    } else {
        LocalScheme::Solution solution = local_scheme.solution(*parameters.initial_solution);
        parameters_a_star.initial_solutions = {solution};
    }
    parameters_a_star.new_solution_callback
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
    a_star_local_search(local_scheme, parameters_a_star);

    return output.algorithm_end(parameters.info);
}


