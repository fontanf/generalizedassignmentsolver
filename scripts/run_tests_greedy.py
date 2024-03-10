import run_tests
import os


commands_greedy_cij = run_tests.generate_commands(
        "--algorithm greedy --desirability cij",
        os.path.join("test_results", "greedy_cij"))

commands_greedy_wij = run_tests.generate_commands(
        "--algorithm greedy --desirability wij",
        os.path.join("test_results", "greedy_wij"))

commands_greedy_cij_wij = run_tests.generate_commands(
        "--algorithm greedy --desirability \"cij*wij\"",
        os.path.join("test_results", "greedy_cij_wij"))

commands_greedy_pij_wij = run_tests.generate_commands(
        "--algorithm greedy --desirability \"-pij/wij\"",
        os.path.join("test_results", "greedy_pij_wij"))

commands_greedy_wij_ti = run_tests.generate_commands(
        "--algorithm greedy --desirability \"wij/ti\"",
        os.path.join("test_results", "greedy_wij_ti"))


commands_greedy_regret_cij = run_tests.generate_commands(
        "--algorithm greedy-regret --desirability cij",
        os.path.join("test_results", "greedy_regret_cij"))

commands_greedy_regret_wij = run_tests.generate_commands(
        "--algorithm greedy-regret --desirability wij",
        os.path.join("test_results", "greedy_regret_wij"))

commands_greedy_regret_cij_wij = run_tests.generate_commands(
        "--algorithm greedy-regret --desirability \"cij*wij\"",
        os.path.join("test_results", "greedy_regret_cij_wij"))

commands_greedy_regret_pij_wij = run_tests.generate_commands(
        "--algorithm greedy-regret --desirability \"-pij/wij\"",
        os.path.join("test_results", "greedy_regret_pij_wij"))

commands_greedy_regret_wij_ti = run_tests.generate_commands(
        "--algorithm greedy-regret --desirability \"wij/ti\"",
        os.path.join("test_results", "greedy_regret_wij_ti"))


commands_mthg_cij = run_tests.generate_commands(
        "--algorithm mthg --desirability cij",
        os.path.join("test_results", "mthg_cij"))

commands_mthg_wij = run_tests.generate_commands(
        "--algorithm mthg --desirability wij",
        os.path.join("test_results", "mthg_wij"))

commands_mthg_cij_wij = run_tests.generate_commands(
        "--algorithm mthg --desirability \"cij*wij\"",
        os.path.join("test_results", "mthg_cij_wij"))

commands_mthg_pij_wij = run_tests.generate_commands(
        "--algorithm mthg --desirability \"-pij/wij\"",
        os.path.join("test_results", "mthg_pij_wij"))

commands_mthg_wij_ti = run_tests.generate_commands(
        "--algorithm mthg --desirability \"wij/ti\"",
        os.path.join("test_results", "mthg_wij_ti"))


commands_mthg_regret_cij = run_tests.generate_commands(
        "--algorithm mthg-regret --desirability cij",
        os.path.join("test_results", "mthg_regret_cij"))

commands_mthg_regret_wij = run_tests.generate_commands(
        "--algorithm mthg-regret --desirability wij",
        os.path.join("test_results", "mthg_regret_wij"))

commands_mthg_regret_cij_wij = run_tests.generate_commands(
        "--algorithm mthg-regret --desirability \"cij*wij\"",
        os.path.join("test_results", "mthg_regret_cij_wij"))

commands_mthg_regret_pij_wij = run_tests.generate_commands(
        "--algorithm mthg-regret --desirability \"-pij/wij\"",
        os.path.join("test_results", "mthg_regret_pij_wij"))

commands_mthg_regret_wij_ti = run_tests.generate_commands(
        "--algorithm mthg-regret --desirability \"wij/ti\"",
        os.path.join("test_results", "mthg_regret_wij_ti"))


commands = ([]
        + commands_greedy_cij
        + commands_greedy_wij
        + commands_greedy_cij_wij
        + commands_greedy_pij_wij
        + commands_greedy_wij_ti
        + commands_greedy_regret_cij
        + commands_greedy_regret_wij
        + commands_greedy_regret_cij_wij
        + commands_greedy_regret_pij_wij
        + commands_greedy_regret_wij_ti
        + commands_mthg_cij
        + commands_mthg_wij
        + commands_mthg_cij_wij
        + commands_mthg_pij_wij
        + commands_mthg_wij_ti
        + commands_mthg_regret_cij
        + commands_mthg_regret_wij
        + commands_mthg_regret_cij_wij
        + commands_mthg_regret_pij_wij
        + commands_mthg_regret_wij_ti
        )


if __name__ == "__main__":
    for command in commands:
        run_tests.run(command)
