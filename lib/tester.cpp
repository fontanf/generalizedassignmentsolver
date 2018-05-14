#include "gap/lib/tester.hpp"

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include <thread>

using namespace gap;

void executeProgram(std::string cmd)
{
    std::cout << cmd << std::endl;
    system(cmd.c_str());
}

void check_sopt(std::string prog, boost::filesystem::path input)
{
    Instance instance(input);
    Profit opt = instance.optimum();

    std::string output_file = "output_file.ini";
    std::string cert_file   = "cert_file.txt";
    std::string cmd = prog
        + " -i" + input.string()
        + " -o" + output_file
        + " -c" + cert_file;
    std::thread worker_rec(executeProgram, cmd);
    worker_rec.join();

    EXPECT_EQ(boost::filesystem::exists(output_file), true);
    if (!boost::filesystem::exists(output_file))
        return;

    EXPECT_EQ(boost::filesystem::exists(cert_file), true);
    if (!boost::filesystem::exists(cert_file))
        return;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(output_file, pt);
    EXPECT_EQ(pt.get<Profit>("Solution.OPT"), opt);
    EXPECT_EQ(instance.check(cert_file),      opt);

    boost::filesystem::remove(output_file);
    boost::filesystem::remove(cert_file);
}

void check_opt(std::string prog, boost::filesystem::path input)
{
    Instance instance(input);
    Profit opt = instance.optimum();

    std::string output_file = "output_file.ini";
    std::string cmd = prog
        + " -i" + input.string()
        + " -o" + output_file;
    std::thread worker_rec(executeProgram, cmd);
    worker_rec.join();

    EXPECT_EQ(boost::filesystem::exists(output_file), true);
    if (!boost::filesystem::exists(output_file))
        return;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(output_file, pt);
    EXPECT_EQ(pt.get<Profit>("Solution.OPT"), opt);

    boost::filesystem::remove(output_file);
}

void test(std::string exec, std::string test)
{
    boost::filesystem::path data_dir = boost::filesystem::current_path();
    data_dir /= boost::filesystem::path("data_tests");

    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(data_dir); itr != end_itr; ++itr) {
        if (itr->path().filename() == "FORMAT.txt")
            continue;
        if (itr->path().filename() == "BUILD")
            continue;
        if (itr->path().extension() != ".txt")
            continue;
        if (test == "sopt") {
            check_sopt(exec, itr->path());
        } else if (test == "opt") {
            check_opt(exec, itr->path());
        }
    }
}
