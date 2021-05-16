#include "solvergreedy.h"

#include "searcherpaths.h"
#include "searchermoves.h"
#include "searcherpoints.h"
#include "searcherboundedpaths.h"
#include "finderpoints.h"

#include <algorithm>    // std::sort

SolverGreedy::SolverGreedy(const Instance &ins, const Options &options) :
    m_ins(ins), m_opt(options), m_sol(ins), m_steps_ahead(1)
{
    /* initialize the first configuration and the solution */
    assert(m_cur_config.empty());
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        m_cur_config.push_back(m_ins.s(r));
        m_sol.add_robot_position(r, m_ins.s(r));
    }
}

/**
 * @brief SolverGreedy::solve
 */
bool SolverGreedy::solve(int space)
{
    std::clog << "Solving with SolverGreedy::solve(time=" << m_opt.time
              << ", space=" << space << ")..." << std::endl;
    m_steps_ahead = m_opt.time;
    if (m_steps_ahead < 3)
        std::cout << "With " << m_steps_ahead << " (< 3) steps, this can loop forever" << std::endl;
    const int limit_steps = 10 * m_ins.makespan_bound(); // dumb bound to avoid loops
    int iterations = 0;
    while (current_distance() >= m_steps_ahead)
    {
        if (space <= 0)
            iteration();
        else
            make_next_step(space);
        m_sol.write("debug", "solvergreedy-dbg.json");
        if (m_sol.looping() || m_sol.makespan() > limit_steps /*|| stable_distance(distances)*/)
        {
            std::cerr << "The solver has possibly entered an infinite loop" << std::endl;
            m_sol.write("Looping");
            return false;
        }
        iterations++;
    }
    make_last_steps();

    std::cout << "Makespan: " << m_sol.makespan() << ", distance: " << m_sol.distance() << std::endl;
    m_sol.write("debug", "solvergreedy-dbg.json");
    return true;
}

/**
 * @brief SolverGreedy::make_last_steps
 * When the robots are close to their targets, we finish the solution with an
 * exact solver.
 */
void SolverGreedy::make_last_steps()
{
    /* Make a new instance with the rest of the problem */
    Instance end_ins;
    for (int i = 0; i < m_ins.nb_obstacles(); i++)
    {
        const Point p = m_ins.obstacle(i);
        end_ins.add_obstacle(p.x, p.y);
    }
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point &start = m_cur_config.at(r);
        const Point &target = m_ins.t(r);
        end_ins.add_robot(start, target);
    }
    /* Use an exact solver */
    FinderPoints solver(end_ins, m_opt);
    int makespan = 0;
    while (!solver.solve(makespan))
        makespan++;
    Solution end_sol = solver.solution();
    /* add the steps to the solution */
    for (int r = 0; r < m_ins.nb_robots(); r++)
        for (int k = 0; k <= end_sol.makespan(); k++)
        {
            const Point p = end_sol.point(r, k);
            m_sol.add_robot_position(r, p);
        }
}


/**
 * @brief SolverGreedy::finished
 * @return True if all robots have reached their respective target
 * @note This function is currently not used
 */
bool SolverGreedy::finished() const
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
        if (m_cur_config.at(r) != m_ins.t(r))
            return false;
    return true;
}

/**
 * @brief SolverGreedy::iteration
 * Compute the next position of the robots. This function solves the MIP problem
 */
void SolverGreedy::iteration()
{
    leading_robots();

    const int max_dist = current_distance();
    const int steps_in_advance = std::min(max_dist, m_steps_ahead);
    SearcherPaths planner(m_ins, m_opt, steps_in_advance);
//    SearcherPoints planner(m_ins, m_opt, steps_in_advance);
//    SearcherMoves planner(m_ins, m_opt, steps_in_advance);
    for (int r = 0; r < m_ins.nb_robots(); r++)
        planner.set_robot_position(r, m_cur_config.at(r));

    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const int cur_dist = m_ins.distance(m_cur_config.at(r), m_ins.t(r));
        for (int i = -steps_in_advance; i <= steps_in_advance; i++)
            for (int j = -steps_in_advance; j <= steps_in_advance; j++)
            {
                const Point p(i,j);
                if (p.dist(Point(0,0)) <= steps_in_advance)
                {
                    const double w = weight(steps_in_advance, r, cur_dist, p);
                    planner.set_weight(r, p, w);
                }
            }
    }

    planner.solve();

    /* Use only the first step */
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        m_sol.add_robot_position(r, planner.robot_position(r, 1));
        m_cur_config.at(r) = planner.robot_position(r, 1);
    }
}

/**
 * @brief SolverGreedy::weight
 * Compute the weight of the paths moving the r-th robot to m_cur_config.at(r) + p
 *
 * There are some special cases:
 * 1. If the endpoint is an obstacle, we assign the weight -1 because it is not a
 *    feasible solution.
 * 2. If the endpoint has infinite (-1) distance, then there is no path to that
 *    endpoint, because it must cross a wall of obstacles.
 * 3. If the endpoint is too much closer to the target, it means that there is no
 *    path to that endpoint.
 *
 * @param steps Length of the paths considered
 * @param dist Current distance from r to its target.
 * @param p relative position of the endpoint
 */
double SolverGreedy::weight(int steps, int r, int dist, Point p) const
{
    if (m_ins.is_obstacle(m_cur_config.at(r) + p))
        return -1; // special case 1

    const int next_dist = m_ins.distance(m_cur_config.at(r) + p, m_ins.t(r));
    if (next_dist < 0)
        return -1; // special case 2
    if (dist - next_dist > steps)
        return -1; // special case 3

    const double w = (dist*dist + 1) * (dist - next_dist);
    return w;
}


/**
 * @brief SolverGreedy::stable_distance
 * @return True if the last N distances are the same.
 * I set N to 6, because I am trying to understand when I am in a loop.
 */
bool SolverGreedy::stable_distance(const std::vector<int> &distances) const
{
    const int n = 6;
    if (distances.size() < n)
        return false;
    const int dist = distances.back();
    for (int i = 1; i < n; i++)
    {
        if (distances.at(distances.size()-1-i) != dist)
            return false;
    }
    return true;
}


/**
 * @brief SolverGreedy::current_distance
 * @return The maximum distance from a robot to its target.
 * If this is small, we can compute less steps in advance
 */
int SolverGreedy::current_distance() const
{
    int max_dist = 0;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const int dist_r = m_ins.distance(m_cur_config.at(r), m_ins.t(r));
        max_dist = std::max(max_dist, dist_r);
    }
    return max_dist;
}

/**
 * @brief SolverGreedy::leading_robots
 * Similar to SolverGreedy::current_distance, but it also print the robots
 * maximizing the longest remaining distance
 */
int SolverGreedy::leading_robots() const
{
    int max_dist = -1;
    std::list<int> leading;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const int dist_r = m_ins.distance(m_cur_config.at(r), m_ins.t(r));
        if (dist_r > max_dist)
        {
            leading.clear();
            leading.push_back(r);
            max_dist = dist_r;
        }
        else if (dist_r == max_dist)
        {
            leading.push_back(r);
        }
    }
    std::cout << "Leading robots at distance " << max_dist << ":";
    for (int r : leading)
        std::cout << " " << r;
    std::cout << std::endl;
    return max_dist;
}


Point SolverGreedy::follow_path(Point p, const std::vector<Point> &moves) const
{
    for (const Point &q : moves)
    {
        p = p.add(q);
    }
    return p;
}

/**
 * @brief SolverGreedy::moves_in_path
 * @return The number of moves in this path.
 * @note The name of this function can mess your mind up.
 */
int SolverGreedy::moves_in_path(const std::vector<Point> &moves) const
{
    int d = 0;
    for (const Point &p : moves)
        if (p != Point(0,0))
            d++;
    return d;
}

/**
 * @brief SolverGreedy::make_next_step
 * Compute the next step of the solution by dividing bounding box in squared
 * regions of size `space` x `space` plus a padding
 *
 * @note The padding must be > 0 because otherwise we restrict the solution to
 * the bounding box and that can isolate robots. It should be at least the number
 * of steps, so that a robot has the possibility to make plan the longest path. (old?)
 *
 * @note We know that each unweighted robot will find a solution if its path is
 * contained in other flap. For this, flap <= space.
 */
void SolverGreedy::make_next_step(int space)
{
    leading_robots();

    const int max_dist = current_distance();
    const int steps = std::min(max_dist, m_steps_ahead);
//    assert(padding >= steps); // necessary for the overlap conditions
    std::vector<Box> regions = sorted_regions(space);
    std::vector<bool> planned_robots(m_ins.nb_robots(), false);
    Solution planned_sol(m_ins);
    for (const auto &region : regions)
    {
        SearcherBoundedPaths planner(m_ins, planned_sol, planned_robots, steps, m_opt);
        planner.set_start(m_cur_config);
        planner.set_bbox(region);
        std::list<int> weighted_robots;
        /* set the weights */
        for (int r = 0; r < m_ins.nb_robots(); r++)
        {
            if (region.inside(m_cur_config.at(r)))
            {
                assert(!planned_robots.at(r));
                const int cur_dist = m_ins.distance(m_cur_config.at(r), m_ins.t(r));
                for (int i = -steps; i <= steps; i++)
                {
                    for (int j = -steps; j <= steps; j++)
                    {
                        const Point p(i,j);
                        if (p.dist(Point(0,0)) <= steps)
                        {
                            const double w = weight(steps, r, cur_dist, p);
                            planner.set_weight(r, p, w);
                        }
                    }
                }
                weighted_robots.push_back(r);
            }
        }

        int flap = space;
        planner.set_flap(flap);
        while (!planner.solve())
        {
            planner.clear_model();
            planner.set_flap(++flap);
            std::clog << "Flap increased to " << flap << std::endl;
            assert(flap <= space); // It is true that there exists a feasible solution if flap == space
        }
        // add robot to the proposed solution
        for (int r : weighted_robots)
        {
            planned_robots.at(r) = true;
            for (int k = 0; k <= steps; k++)
            {
                const Point p = planner.solution().point(r, k);
                planned_sol.add_robot_position(r, p);
            }
        }
    }

    /* Use only the first step */
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        m_sol.add_robot_position(r, planned_sol.point(r, 1));
        m_cur_config.at(r) = planned_sol.point(r, 1);
    }
}

/**
 * @brief SolverGreedy::sorted_regions
 * Sort the regions by the maximum distance of the robots in them.
 */
std::vector<Box> SolverGreedy::sorted_regions(int space) const
{
    /* Make the bounding box */
    Box bbox;
    for (int r = 0; r < m_ins.nb_robots(); r++)
        bbox.include(m_cur_config.at(r));
    /* Make the regions */
    std::vector<Box> regions;
    for (int x = bbox.p1.x; x <= bbox.p2.x; x += space)
    {
        for (int y = bbox.p1.y; y <= bbox.p2.y; y += space)
        {
            const Box cur_box(Point(x,y), m_sol.makespan(), Point(x+space-1, y+space-1), m_sol.makespan() + m_steps_ahead);
            regions.push_back(cur_box);
        }
    }
    /* find max distance to target in each region */
    std::vector<int> dist(regions.size(), 0);
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const int i = (m_cur_config.at(r).x - bbox.p1.x) / space;
        const int j = (m_cur_config.at(r).y - bbox.p1.y) / space;
        const int n = std::ceil((double)bbox.size().y / space);
        const int index = n*i + j;
        assert(regions.at(index).inside(m_cur_config.at(r)));
        const int dist_r = m_ins.distance(m_cur_config.at(r), m_ins.t(r));
        dist.at(index) = std::max(dist.at(index), dist_r);
    }
    /* sort regions */
    std::vector<std::pair<Box, int>> zipped(regions.size());
    for (std::size_t i = 0; i < regions.size(); ++i)
        zipped.at(i) = std::pair<Box, int>(regions.at(i), dist.at(i));

    std::sort(zipped.begin(), zipped.end(), [](std::pair<Box, int> a, std::pair<Box, int> b) {
            return a.second > b.second;
        });
    for (std::size_t i = 0; i < regions.size(); ++i)
        regions.at(i) = zipped.at(i).first;
    return regions;
}
