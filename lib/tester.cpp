#include "generalizedassignment/lib/tester.hpp"

#include "generalizedassignment/lib/solution.hpp"
#include "generalizedassignment/lib/generator.hpp"

using namespace generalizedassignment;

bool test(const Instance& ins, std::vector<Output (*)(Instance&)> fs, TestType tt)
{
    bool feasible = (ins.optimal_solution() == NULL)? false: ins.optimal_solution()->feasible();
    Cost opt = (ins.optimal_solution() == NULL)? -1: ins.optimum();
    for (auto f: fs) {
        Instance ins_tmp = ins;
        Output output = f(ins_tmp);
        if (ins.optimal_solution() == NULL && opt == -1) {
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
                EXPECT_GE(output.lower_bound, ins.bound());
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
Instance generalizedassignment::test_instance(Cpt i)
{
    std::cout << "i " << i << std::endl;
    switch (i) {
    case 0: { // 0 item
        Instance ins(0);
        Solution sol(ins);
        ins.set_optimal_solution(sol);
        return ins;
    } case 1: { // 1 agent, 1 item, opt 10
        Instance ins(1);
        ins.set_capacities({5});
        ins.add_item({{4, 10}});
        Solution sol(ins, v{{0}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 2: { // 1 agent, 2 item, opt 18
        Instance ins(1);
        ins.set_capacities({5});
        ins.add_item({{3, 10}});
        ins.add_item({{2, 8}});
        Solution sol(ins, v{{0, 1}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 3: { // 2 agent, 1 items, opt 10
        Instance ins(2);
        ins.set_capacities({5, 7});
        ins.add_item({{5, 10}, {4, 11}});
        Solution sol(ins, v{{0}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 4: { // 2 agent, 2 items, opt 15
        Instance ins(2);
        ins.set_capacities({5, 7});
        ins.add_item({{3, 11}, {4, 10}});
        ins.add_item({{2, 6}, {3, 5}});
        Solution sol(ins, {{}, {0, 1}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 5: { // 2 agents, 2 items, opt 16
        Instance ins(2);
        ins.set_capacities({5, 7});
        ins.add_item({{3, 11}, {4, 10}});
        ins.add_item({{4, 7}, {5, 5}});
        Solution sol(ins, v{{0}, {1}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 6: { // 2 agent, 3 items, opt 23
        Instance ins(2);
        ins.set_capacities({6, 9});
        ins.add_item({{1, 13}, {2, 10}});
        ins.add_item({{3, 9}, {4, 8}});
        ins.add_item({{2, 7}, {3, 5}});
        Solution sol(ins, {{}, {0, 1, 2}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 7: { // 2 agent, 3 items, opt 24
        Instance ins(2);
        ins.set_capacities({5, 7});
        ins.add_item({{1, 13}, {2, 10}});
        ins.add_item({{3, 9}, {4, 8}});
        ins.add_item({{2, 7}, {3, 5}});
        Solution sol(ins, {{1}, {0, 2}});
        ins.set_optimal_solution(sol);
        return ins;
    } case 8: { // 3 agent, 5 items
        Instance ins(3);
        ins.set_capacities({10, 12, 11});
        ins.add_item({{1, 4}, {2, 6}, {3, 7}});
        ins.add_item({{2, 8}, {3, 6}, {4, 5}});
        ins.add_item({{3, 10}, {4, 8}, {5, 6}});
        ins.add_item({{4, 9}, {5, 8}, {6, 7}});
        ins.add_item({{5, 12}, {6, 11}, {7, 9}});
        Solution sol(ins, {{0}, {2, 3}, {1, 4}});
        ins.set_optimal_solution(sol);
        return ins;
    } default: {
        return Instance(0);
    }
    }
}

class TestInstances: public Instances
{

public:

    Instance next()
    {
        i++;
        if (i == 8)
            end = true;
        return test_instance(i);
    }

private:

    Cpt i = -1;

};

void test(Instances& inss, std::vector<Output (*)(Instance&)> fs, TestType tt)
{
    for (;;) {
        Instance ins(inss.next());
        bool b = test(ins, fs, tt);
        if (!b) {
            std::cout << "error" << std::endl;
            return;
        }
        if (inss.end)
            break;
    }
    std::cout << "ok" << std::endl;
}

void generalizedassignment::test(InstacesType it, std::vector<Output (*)(Instance&)> fs, TestType tt)
{
    if (it == TEST) {
        TestInstances ti;
        test(ti, fs, tt);
    }
}

