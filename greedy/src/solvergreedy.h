#ifndef SOLVERGREEDY_H
#define SOLVERGREEDY_H

#include "instance.h"
#include "solution.h"

/**
 * @brief The SolverGreedy class
 * Greedy mathod for the the **makespan** problem.
 * We push each robot to its target. Priority is given to more distant robots, which
 * are allowed to block other robots.
 *
 * @todo It does not reach the final solution. Where the robots are at distance 3, they start doing whatever and
 * I don't see how to encourage them to go to the target in the first steps.
 */
class SolverGreedy
{
public:
    SolverGreedy(const Instance &ins, const Options &options);
    bool solve(int space = 0);
    inline Solution solution() const { return m_sol; }

private:
    bool finished() const;
    void iteration();
    void make_last_steps();
    int current_distance() const;
    int leading_robots() const;
    double weight(int steps, int r, int dist, Point p) const;
    bool stable_distance(const std::vector<int> &distances) const;
    Point follow_path(Point p, const std::vector<Point> &moves) const;
    int moves_in_path(const std::vector<Point> &moves) const;

    void make_next_step(int space);
    std::vector<Box> sorted_regions(int space) const;



private:
    const Instance &m_ins;
    const Options &m_opt;
    Solution m_sol;
    std::vector<Point> m_cur_config; // current position of the robots
    int m_steps_ahead; // number of steps ahead used in the one-step-planner
};

#endif // SOLVERGREEDY_H
