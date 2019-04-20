#include "gap/lib/tester.hpp"

#include "gap/lib/solution.hpp"
#include "gap/lib/generator.hpp"

using namespace gap;

bool test(const Instance& ins, std::vector<Value (*)(Instance&)> fs, TestType tt)
{
    Value opt = ins.optimum();
    for (auto f: fs) {
        Instance ins_tmp = ins;
        Value val = f(ins_tmp);
        if (opt == -1)
            opt = val;
        if (tt == OPT) {
            EXPECT_EQ(val, opt);
            if (val != opt)
                return false;
        } else if (tt == UB) {
            EXPECT_GE(val, opt);
            if (val < opt)
                return false;
        } else if (tt == LB) {
            EXPECT_LE(val, opt);
            if (val > opt)
                return false;
        }
    }
    return true;
}

class Instances
{
public:
    virtual Instance next() = 0;
};

class TestInstances: public Instances
{

public:

    Instance next()
    {
        i++;
        return test_instance(i);
    }

    Instance test_instance(Cpt i)
    {
        switch (i) {
        case 0: { // 0 item
            Instance ins(0, 0);
            Solution sol(ins);
            ins.set_optimal_solution(sol);
            return ins;
        } case 3: { // 2 agent, 1 items, opt 10
            Instance ins(2, 1);
            ins.add_items(1);
            ins.set_capacity(0, 5);
            ins.set_capacity(1, 7);
            ins.set_alternative(0, 0, 5, 10);
            ins.set_alternative(0, 1, 4, 11);
            Solution sol(ins);
            sol.set(0, 0);
            ins.set_optimal_solution(sol);
            return ins;
        } case 4: { // 2 agent, 2 items, opt 15
            Instance ins(2, 2);
            ins.add_items(2);
            ins.set_capacity(0, 5);
            ins.set_capacity(1, 7);
            ins.set_alternative(0, 0, 3, 11);
            ins.set_alternative(0, 1, 4, 10);
            ins.set_alternative(1, 0, 2, 6);
            ins.set_alternative(1, 1, 3, 5);
            Solution sol(ins);
            sol.set(0, 1);
            sol.set(1, 1);
            ins.set_optimal_solution(sol);
            return ins;
        } case 5: { // 2 agents, 2 items, opt 16
            Instance ins(2, 2);
            ins.add_items(2);
            ins.set_capacity(0, 5);
            ins.set_capacity(1, 7);
            ins.set_alternative(0, 0, 3, 11);
            ins.set_alternative(0, 1, 4, 10);
            ins.set_alternative(1, 0, 4, 7);
            ins.set_alternative(1, 1, 5, 5);
            Solution sol(ins);
            sol.set(0, 0);
            sol.set(1, 1);
            ins.set_optimal_solution(sol);
            return ins;
        } case 6: { // 2 agent, 3 items, opt 23
            Instance ins(2, 3);
            ins.add_items(3);
            ins.set_capacity(0, 6);
            ins.set_capacity(1, 9);
            ins.set_alternative(0, 0, 1, 13);
            ins.set_alternative(0, 1, 2, 10);
            ins.set_alternative(1, 0, 3, 9);
            ins.set_alternative(1, 1, 4, 8);
            ins.set_alternative(2, 0, 2, 7);
            ins.set_alternative(2, 1, 3, 5);
            Solution sol(ins);
            sol.set(0, 1);
            sol.set(1, 1);
            sol.set(2, 1);
            ins.set_optimal_solution(sol);
            return ins;
        } case 7: { // 2 agent, 3 items, opt 24
            Instance ins(2, 3);
            ins.add_items(3);
            ins.set_capacity(0, 5);
            ins.set_capacity(1, 7);
            ins.set_alternative(0, 0, 1, 13);
            ins.set_alternative(0, 1, 2, 10);
            ins.set_alternative(1, 0, 3, 9);
            ins.set_alternative(1, 1, 4, 8);
            ins.set_alternative(2, 0, 2, 7);
            ins.set_alternative(2, 1, 3, 5);
            Solution sol(ins);
            sol.set(0, 1);
            sol.set(1, 0);
            sol.set(2, 1);
            ins.set_optimal_solution(sol);
            return ins;
        } case 9: { // 3 agent, 5 items
            Instance ins(3, 5);
            ins.add_items(5);
            ins.set_capacity(0, 10);
            ins.set_capacity(1, 12);
            ins.set_capacity(2, 11);
            ins.set_alternative(0, 0, 1, 4);
            ins.set_alternative(0, 1, 2, 6);
            ins.set_alternative(0, 2, 3, 7);
            ins.set_alternative(1, 0, 2, 8);
            ins.set_alternative(1, 1, 3, 6);
            ins.set_alternative(1, 2, 4, 5);
            ins.set_alternative(2, 0, 3, 10);
            ins.set_alternative(2, 1, 4, 8);
            ins.set_alternative(2, 2, 5, 6);
            ins.set_alternative(3, 0, 4, 9);
            ins.set_alternative(3, 1, 5, 8);
            ins.set_alternative(3, 2, 6, 7);
            ins.set_alternative(4, 0, 5, 12);
            ins.set_alternative(4, 1, 6, 11);
            ins.set_alternative(4, 2, 7, 9);
            return ins;
        } default: {
            return Instance(0, 0);
        }
        }
    }

private:

    Cpt i = 0;

};

void test(Instances& inss, std::vector<Value (*)(Instance&)> fs, TestType tt)
{
    for (;;) {
        Instance ins(inss.next());
        bool b = test(ins, fs, tt);
        if (!b) {
            std::cout << "error" << std::endl;
            return;
        }
        if (ins.item_number() == 0)
            break;
    }
    std::cout << "ok" << std::endl;
}

void gap::test(InstacesType it, std::vector<Value (*)(Instance&)> fs, TestType tt)
{
    if (it == TEST) {
        TestInstances ti;
        test(ti, fs, tt);
    }
}

