#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include "localsearchsolver/a_star.hpp"

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

    inline ItemIdx&       item_number(GlobalCost& global_cost) const { return std::get<0>(global_cost); }
    inline Weight&         overweight(GlobalCost& global_cost) const { return std::get<1>(global_cost); }
    inline Cost&                 cost(GlobalCost& global_cost) const { return std::get<2>(global_cost); }
    inline ItemIdx  item_number(const GlobalCost& global_cost) const { return std::get<0>(global_cost); }
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
        ItemIdx item_number = 0;
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
        for (ItemIdx j = 0; j < instance_.item_number(); ++j) {
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
        const generalizedassignmentsolver::Solution* initial_solution = NULL;
    };

    LocalScheme(
            const Instance& instance,
            Parameters parameters):
        instance_(instance),
        parameters_(parameters),
        items_(instance.item_number()),
        agents_(instance.agent_number())
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
        solution.agents.resize(instance_.item_number(), -1);
        solution.weights.resize(instance_.agent_number(), 0);
        return solution;
    }

    inline Solution initial_solution(
            Counter,
            std::mt19937_64& generator) const
    {
        Solution solution = empty_solution();
        if (parameters_.initial_solution != nullptr) {
            for (ItemIdx j = 0; j < instance_.item_number(); ++j) {
                AgentIdx i = parameters_.initial_solution->agent(j);
                if (i != -1)
                    add(solution, j, i);
            }
        }

        std::uniform_int_distribution<ItemIdx> d(0, instance_.agent_number() - 1);
        for (ItemIdx j = 0; j < instance_.item_number(); ++j) {
            if (solution.agents[j] != -1)
                continue;
            AgentIdx i = d(generator);
            add(solution, j, i);
        }
        return solution;
    }

    /*
     * Solution properties.
     */

    inline GlobalCost global_cost(const Solution& solution) const
    {
        return {
            -solution.item_number,
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

    inline std::vector<Move> perturbations(Solution& solution) const
    {
        std::vector<Move> moves;
        for (ItemIdx j = 0; j < instance_.item_number(); ++j) {
            AgentIdx i_old = solution.agents[j];
            remove(solution, j);
            for (AgentIdx i = 0; i < instance_.agent_number(); ++i) {
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
        os << "item number: " << solution.item_number << std::endl;
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
        assert(j < instance_.item_number());
        assert(i >= 0);
        assert(i < instance_.agent_number());
        assert(solution.agents[j] == -1);
        // Update item_number.
        solution.item_number++;
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
        // Update item_number.
        solution.item_number--;
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
        // Update item_number.
        item_number(gc)--;
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
    parameters_local_scheme.initial_solution = parameters.initial_solution;
    LocalScheme local_scheme(instance, parameters_local_scheme);

    // Run A*.
    AStarOptionalParameters<LocalScheme> parameters_a_star;
    parameters_a_star.info.set_verbose(false);
    parameters_a_star.info.set_timelimit(parameters.info.remaining_time());
    parameters_a_star.thread_number_1 = 1;
    parameters_a_star.thread_number_2 = parameters.thread_number;
    parameters_a_star.initial_solution_ids = std::vector<Counter>(
            parameters_a_star.thread_number_2, 0);
    parameters_a_star.new_solution_callback
        = [&instance, &parameters, &output](
                const LocalScheme::Solution& solution)
        {
            Solution sol(instance);
            for (ItemIdx j = 0; j < instance.item_number(); ++j) {
                AgentIdx i = solution.agents[j];
                sol.set(j, i);
            }
            std::stringstream ss;
            output.update_solution(sol, ss, parameters.info);
        };
    a_star(local_scheme, parameters_a_star);

    return output.algorithm_end(parameters.info);
}


