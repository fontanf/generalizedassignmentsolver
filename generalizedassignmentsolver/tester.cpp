#include "generalizedassignmentsolver/tester.hpp"

#include "generalizedassignmentsolver/solution.hpp"
#include "generalizedassignmentsolver/generator.hpp"

using namespace generalizedassignmentsolver;

bool test(const Instance& instance, std::vector<Output (*)(Instance&)> fs, TestType tt)
{
    bool feasible = (instance.optimal_solution() == NULL)? false: instance.optimal_solution()->feasible();
    Cost opt = (instance.optimal_solution() == NULL)? -1: instance.optimum();
    for (auto f: fs) {
        Instance instance_tmp = instance;
        Output output = f(instance_tmp);
        if (instance.optimal_solution() == NULL && opt == -1) {
            feasible = output.feasible();
            opt = output.solution.cost();
            continue;
        }
        if (tt == SOPT) {
            if (feasible) {
                EXPECT_EQ(output.lower_bound, opt);
                EXPECT_TRUE(output.solution.feasible());
                EXPECT_EQ(output.lower_bound, output.solution.cost());
            } else {
                EXPECT_GE(output.lower_bound, instance.bound());
            }
        } else if (tt == LB) {
            if (feasible) {
                EXPECT_LE(output.lower_bound, opt);
            }
        } else if (tt == UB) {
            if (feasible) {
                EXPECT_GE(output.solution.profit(), opt);
            }
        }
    }
    return true;
}

class Instances
{
public:
    virtual Instance next() = 0;
    bool end = false;
};

typedef const std::vector<std::vector<ItemIdx>>& v;
class TestInstances: public Instances
{

public:

    Instance test_instance(Counter i)
    {
        std::cout << "i " << i << std::endl;
        switch (i) {
        case 0: { // 0 item
            Instance instance(0);
            Solution sol(instance);
            instance.set_optimal_solution(sol);
            return instance;
        } case 1: { // 1 agent, 1 item, opt 10
            Instance instance(1);
            instance.set_capacities({5});
            instance.add_item({{4, 10}});
            Solution sol(instance, v{{0}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 2: { // 1 agent, 2 item, opt 18
            Instance instance(1);
            instance.set_capacities({5});
            instance.add_item({{3, 10}});
            instance.add_item({{2, 8}});
            Solution sol(instance, v{{0, 1}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 3: { // 2 agent, 1 items, opt 10
            Instance instance(2);
            instance.set_capacities({5, 7});
            instance.add_item({{5, 10}, {4, 11}});
            Solution sol(instance, v{{0}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 4: { // 2 agent, 2 items, opt 15
            Instance instance(2);
            instance.set_capacities({5, 7});
            instance.add_item({{3, 11}, {4, 10}});
            instance.add_item({{2, 6}, {3, 5}});
            Solution sol(instance, {{}, {0, 1}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 5: { // 2 agents, 2 items, opt 16
            Instance instance(2);
            instance.set_capacities({5, 7});
            instance.add_item({{3, 11}, {4, 10}});
            instance.add_item({{4, 7}, {5, 5}});
            Solution sol(instance, v{{0}, {1}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 6: { // 2 agent, 3 items, opt 23
            Instance instance(2);
            instance.set_capacities({6, 9});
            instance.add_item({{1, 13}, {2, 10}});
            instance.add_item({{3, 9}, {4, 8}});
            instance.add_item({{2, 7}, {3, 5}});
            Solution sol(instance, {{}, {0, 1, 2}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 7: { // 2 agent, 3 items, opt 24
            Instance instance(2);
            instance.set_capacities({5, 7});
            instance.add_item({{1, 13}, {2, 10}});
            instance.add_item({{3, 9}, {4, 8}});
            instance.add_item({{2, 7}, {3, 5}});
            Solution sol(instance, {{1}, {0, 2}});
            instance.set_optimal_solution(sol);
            return instance;
        } case 8: { // 3 agent, 5 items
            Instance instance(3);
            instance.set_capacities({10, 12, 11});
            instance.add_item({{1, 4}, {2, 6}, {3, 7}});
            instance.add_item({{2, 8}, {3, 6}, {4, 5}});
            instance.add_item({{3, 10}, {4, 8}, {5, 6}});
            instance.add_item({{4, 9}, {5, 8}, {6, 7}});
            instance.add_item({{5, 12}, {6, 11}, {7, 9}});
            Solution sol(instance, {{0}, {2, 3}, {1, 4}});
            instance.set_optimal_solution(sol);
            return instance;
        } default: {
            return Instance(0);
        }
        }
    }

    Instance next()
    {
        i++;
        if (i == 8)
            end = true;
        return test_instance(i);
    }

private:

    Counter i = -1;

};

void test(Instances& instances, std::vector<Output (*)(Instance&)> fs, TestType tt)
{
    while (!instances.end) {
        Instance instance(instances.next());
        bool b = test(instance, fs, tt);
        if (!b) {
            std::cout << "error" << std::endl;
            return;
        }
    }
    std::cout << "ok" << std::endl;
}

void generalizedassignmentsolver::test(
        InstacesType instances_type,
        std::vector<Output (*)(Instance&)> algorithms,
        TestType test_type)
{
    switch (instances_type) {
    case TEST: {
        TestInstances ti;
        test(ti, algorithms, test_type);
        break;
    } default: {
    }
    }
}

