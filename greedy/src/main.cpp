#include <iostream>
#include <string>
#include <cassert>

#include "instance.h"
#include "solvergreedy.h"


/**
 * @brief usage
 * Print usage description
 */
void usage(char** argv)
{
    std::cout << "Usage: " << argv[0] << " [OPTIONS] <instance>" << std::endl;
    std::cout << "<instance>:" << std::endl;
    std::cout << "    instance file. Something like small_000.instance.json" << std::endl;
    std::cout << "--time <n>, -t:" << std::endl;
    std::cout << "    time parameter. n = 3 by default" << std::endl;
    std::cout << "--space <n>, -s:" << std::endl;
    std::cout << "    space parameter. n = 0 by default" << std::endl;
    std::cout << "--time-limit <n>, -tl:" << std::endl;
    std::cout << "    solve each LP problem it n seconds. n = 300 by default" << std::endl;
    std::cout << "--reverse, -r:" << std::endl;
    std::cout << "    swap start and target positions" << std::endl;
}


Options read_options(int argc, char** argv)
{
    Options options;
    options.time = 3;
    options.space = 0;
    options.time_limit = 300;
    for (int i = 1; i < argc; i++)
    {
        const std::string arg(argv[i]);
        if (arg.compare("--time") == 0 || arg.compare("-t") == 0)
        {
            if (i+1 < argc)
            {
                options.time = std::stoi(argv[++i]);
                if (options.time <= 0)
                    std::cerr << "Warning: the time parameter is not positive" << std::endl;
            }
            else { std::cerr << argv[i] << " option needs an int." << std::endl; exit(EXIT_FAILURE); }
        }
        else if (arg.compare("--space") == 0 || arg.compare("-s") == 0)
        {
            if (i+1 < argc)
            {
                options.space = std::stoi(argv[++i]);
                if (options.space <= 0)
                    std::cerr << "Warning: the space parameter is not positive" << std::endl;
            }
            else { std::cerr << argv[i] << " option needs an int." << std::endl; exit(EXIT_FAILURE); }
        }
        else if (arg.compare("--time-limit") == 0 || arg.compare("-tl") == 0)
        {
            if (i+1 < argc)
                options.time_limit = std::stoi(argv[++i]);
            else { std::cerr << argv[i] << " option needs an int." << std::endl; exit(EXIT_FAILURE); }
        }
        else if (arg.compare("--reverse") == 0 || arg.compare("-r") == 0)
        {
            options.reverse = true;
        }
        else
        {
            options.instance = arg;
        }
    }
    return options;
}

/**
 * @brief greedy_solver
 * Greedy solver for makespan
 */
void greedy_solver(Options options)
{
    Instance ins(options.instance);
    if (options.reverse)
        ins.reverse();
    SolverGreedy solver(ins, options);
    bool b = solver.solve(options.space); assert(b);
    Solution sol = solver.solution();
    sol.reduce_makespan();
    sol.write(options.comment());
}


int main(int argc, char** argv)
{
    if (argc < 2)
    {
        usage(argv);
        return 0;
    }
    const Options options = read_options(argc, argv);

    greedy_solver(options);

    return EXIT_SUCCESS;
}
