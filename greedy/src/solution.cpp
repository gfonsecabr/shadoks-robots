#include "solution.h"

#include <iostream>
#include <fstream>
#include <algorithm>    // std::reverse
#include <cassert>
#include <sstream>      // std::ostringstream
#include <iomanip>      // std::put_time
#include <unistd.h>     // gethostname

#include "include/rapidjson/document.h"
#include "include/rapidjson/istreamwrapper.h"
#include "include/rapidjson/writer.h"
#include "include/rapidjson/stringbuffer.h"
#include "include/rapidjson/ostreamwrapper.h"

Solution::Solution(const Instance &ins) :
    m_ins(ins), m_steps(ins.nb_robots())
{
}



Solution::Solution(const Instance &ins, const std::string &filename) :
    Solution(ins)
{
    read(filename);
    std::clog << "Solution " << filename << " loaded" << std::endl;
    check();
}


void Solution::clear_steps()
{
    m_steps.clear();
    m_steps.resize(m_ins.nb_robots());
}


void Solution::clear_steps(int r)
{
    assert(0 <= r && r < m_ins.nb_robots());
    m_steps.at(r).clear();
}


void Solution::set_steps(const std::vector<std::vector<Point>> &steps)
{
    m_steps = steps;
}

/**
 * @brief Solution::read
 * Read a solution from a file.
 * @note It assumes that robot indices start from 0.
 */
void Solution::read(const std::string &filename)
{
    std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
    if (!in.is_open())
    {
        std::cerr << "Error reading " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    rapidjson::IStreamWrapper isw {in};

    rapidjson::Document doc {};
    doc.ParseStream(isw);

    if (doc.HasParseError())
    {
        std::cerr << "Error  : " << doc.GetParseError()  << '\n'
                  << "Offset : " << doc.GetErrorOffset() << '\n';
        exit(EXIT_FAILURE);
    }

    assert(doc["instance"].GetString() == m_ins.name());

    for (int r = 0; r < m_ins.nb_robots(); r++)
        add_robot_position(r, m_ins.s(r));
    const rapidjson::Value& steps = doc["steps"];
    assert(steps.IsArray());
    assert((int)m_steps.size() == m_ins.nb_robots());
    for (rapidjson::SizeType k = 0; k < steps.Size(); k++)
    {
        std::vector<bool> moving_robots(m_ins.nb_robots(), false);
        for (rapidjson::Value::ConstMemberIterator itr = steps[k].MemberBegin(); itr != steps[k].MemberEnd(); ++itr)
        {
            const int r = std::stoi(itr->name.GetString());
            moving_robots.at(r) = true;
            const Point p = point(r, k);
            const std::string move = itr->value.GetString();
            if (move == "S") add_robot_position(r, Point(p.x,   p.y-1));
            if (move == "N") add_robot_position(r, Point(p.x,   p.y+1));
            if (move == "W") add_robot_position(r, Point(p.x-1, p.y  ));
            if (move == "E") add_robot_position(r, Point(p.x+1, p.y  ));
        }
        for (int r = 0; r < m_ins.nb_robots(); r++)
            if (!moving_robots.at(r))
                add_robot_position(r, point(r, k));
    }
}


void Solution::add_robot_position(int r, Point p)
{
    assert(0 <= r && r < m_ins.nb_robots());
    m_steps.at(r).push_back(p);
}


void Solution::set_robot_position(int r, int k, Point p)
{
    assert(0 <= r && r <= m_ins.nb_robots());
    assert(0 <= k && k < nb_steps(r));
    m_steps.at(r).at(k) = p;
}


void Solution::print() const
{
    for (int r = 0; r < m_ins.nb_robots(); ++r)
    {
        std::cout << "r = " << r << ": ";
        for (auto it = m_steps.at(r).cbegin(); it != m_steps.at(r).cend(); ++it)
            std::cout << "(" << it->x << ", " << it->y << ") ";
        std::cout << std::endl;
    }
}

/**
 * @brief Solution::write
 * Write the solution into a JSON file
 */
void Solution::write(std::string comment, std::string filename, bool do_check) const
{
    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    struct std::tm * ptm = std::localtime(&tt);

    const bool good = solved();

    if (filename.empty())
    {
        std::ostringstream oss;
        if (m_ins.name().empty()) oss << "unnamed";
        else                      oss << m_ins.name();
        oss << ".agl." << std::put_time(ptm, "%Y%m%d-%H%M%S");
        if (!good) oss << ".nosol.json";
        else       oss << ".sol.json";
        filename = oss.str();
    }

    std::cout << "-- Writing solution to " << filename << "..." << std::endl;
    std::ofstream file(filename.c_str(), std::ifstream::out | std::ifstream::binary);

    file << "{" << std::endl;
    file << "\t\"instance\": \"" << m_ins.name() << "\"," << std::endl;

    file << "\t\"meta\": {" << std::endl;
    file << "\t\t\"author\": \"Aldo Gonzalez-Lorenzo\"," << std::endl;
    file << "\t\t\"date\": \"" << std::put_time(ptm,"%F %T") << "\"," << std::endl;
    char hn[80]; gethostname(hn, 80);
    file << "\t\t\"host\": \"" << std::string(hn) << "\"," << std::endl;
    file << "\t\t\"computation_sec\": " << m_ins.elapsed_sec() << "," << std::endl;
    file << "\t\t\"comment\": \"" << comment << "\"," << std::endl;
    file << "\t\t\"solved\": " << std::boolalpha << good << "," << std::endl;
    file << "\t\t\"reversed\": " << std::boolalpha << m_ins.is_reversed() << "," << std::endl;
    file << "\t\t\"makespan\": " << makespan() << "," << std::endl;
    file << "\t\t\"makespan_lb\": " << m_ins.makespan_bound() << "," << std::endl;
    file << "\t\t\"distance\": " << distance() << "," << std::endl;
    file << "\t\t\"distance_lb\": " << m_ins.distance_bound() << "" << std::endl;
    file << "\t}," << std::endl;

    file << "\t\"steps\": [" << std::endl;
    const int nb_steps = m_steps.at(0).size();
    for (int k = 0; k < nb_steps-1; ++k)
    {
        const std::vector<std::string> step = moves(k);
        file << "\t\t{ ";
        for (std::size_t i = 0; i < step.size(); i += 2)
        {
            file << "\"" << step.at(i) << "\": \"" << step.at(i+1) << "\"";
            if (i+2 < step.size()) file << ", ";
        }
        file << " }";
        if (k+1 < nb_steps-1) file << ",";
        file << std::endl;
    }
    file << "\t]" << std::endl;

    file << "}" << std::endl;
    file.close();
    if (do_check)
        assert(check());
}


/**
 * @brief Solution::steps
 * Helper function for Solution::write. Given a time step @param k, extract the moves
 * @note If the instance is reversed, I re reverse the paths here
 */
std::vector<std::string> Solution::moves(int k) const
{
    assert(0 <= k && k < (int)m_steps.front().size());
    int next_k = k+1;
    if (m_ins.is_reversed())
    {
        k = (int)m_steps.front().size() - 1 - k;
        next_k = k - 1;
    }

    std::vector<std::string> step;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point p1 = point(r, k);
        const Point p2 = point(r, next_k);
        const std::string id = std::to_string(r);
        if      (p2.x < p1.x && p2.y == p1.y) { step.push_back(id); step.push_back("W"); }
        else if (p2.x > p1.x && p2.y == p1.y) { step.push_back(id); step.push_back("E"); }
        else if (p2.x == p1.x && p2.y < p1.y) { step.push_back(id); step.push_back("S"); }
        else if (p2.x == p1.x && p2.y > p1.y) { step.push_back(id); step.push_back("N"); }
    }
    return step;
}


/**
 * @brief Solution::reduce_makespan
 * Remove the steps where no robot moves
 */
void Solution::reduce_makespan()
{
    int steps_reduced = 0;
    std::vector<std::vector<Point>> steps(m_ins.nb_robots());
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        steps.at(r).push_back(m_steps.at(r).at(0));
    }
    for (int k = 1; k <= makespan(); k++)
    {
        bool same = true;
        for (int r = 0; r < m_ins.nb_robots() && same; r++)
        {
            same = (m_steps.at(r).at(k) == m_steps.at(r).at(k-1));
        }
        if (!same)
        {
            for (int r = 0; r < m_ins.nb_robots(); r++)
            {
                steps.at(r).push_back(m_steps.at(r).at(k));
            }
        }
        else
        {
            steps_reduced++;
        }
    }
    m_steps = steps;
    std::clog << "Steps reduced: " << steps_reduced << std::endl;
}

/**
 * @brief Solution::reverse
 * Reverse the paths
 */
void Solution::reverse()
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        std::reverse(m_steps.at(r).begin(), m_steps.at(r).end());
    }
}


/**
 * @brief Solution::extend
 * Append the steps of `sol` to this solution
 */
void Solution::extend(const Solution &sol)
{
    for (int k = 0; k <= sol.makespan(); ++k)
    {
        for (int r = 0; r < m_ins.nb_robots(); r++)
        {
            add_robot_position(r, sol.point(r, k));
        }
    }
}


/**
 * @brief Solution::check
 * @return True iff the solution is valid (4-neighbors, overlaps, volume, etc.)
 * @todo Not done yet
 */
bool Solution::check() const
{
    // correct number of robots
    assert((int)m_steps.size() == m_ins.nb_robots());

    // same number of steps for each robot
    for (int r = 0; r < m_ins.nb_robots(); r++)
        assert((int)m_steps.at(r).size() == makespan()+1);

    // continuity
    for (int r = 0; r < m_ins.nb_robots(); r++)
        for (int k = 0; k <= makespan()-1; k++)
            assert(point(r, k).dist(point(r, k+1)) <= 1);

    // no full overlap
    for (int k = 0; k <= makespan(); ++k)
    {
        std::set<Point> occupied; // positions occupied at time step k
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            assert(occupied.count(point(r, k)) == 0);
            occupied.insert(point(r, k));
        }
    }
    // no partial overlap
    for (int k = 0; k <= makespan()-1; ++k)
    {
        std::map<Point, int> occupied;
        for (int r = 0; r < m_ins.nb_robots(); ++r)
            occupied[point(r, k)] = r;
        for (int r1 = 0; r1 < m_ins.nb_robots(); ++r1)
        {
            const Point &p1 = point(r1, k  );
            const Point &q1 = point(r1, k+1);
            if (p1 == q1 || occupied.count(q1) == 0)
                continue;
            const int r2 = occupied.at(q1);
            const Point &p2 = point(r2, k  );
            const Point &q2 = point(r2, k+1);
            assert((p1 - p2) == (q1 - q2));
        }
    }
    // obstacles
    for (int r = 0; r < m_ins.nb_robots(); ++r)
        for (int k = 0; k <= makespan(); ++k)
            assert(!m_ins.is_obstacle(point(r, k)));
    std::clog << "Solution checked" << std::endl;
    return true;
}

/**
 * @brief Solution::solved
 * @return The robots reach their targets
 */
bool Solution::solved() const
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        if (point(r, makespan()) != m_ins.t(r))
            return false;
    }
    return true;
}

/**
 * @brief Solution::print_race
 * For each step, print the robots sorted by distance to their target
 */
void Solution::print_race() const
{
    std::cout << "-- Race of current solution" << std::endl;
    for (int k = 0; k <= makespan(); k++)
    {
        std::vector<std::list<int>> dist(250); // this is enough for the challenge instances
        for (int r = 0; r < nb_robots(); r++)
        {
            const int d_r = m_ins.distance(point(r, k), m_ins.t(r));
            dist.at(d_r).push_back(r);
        }
        std::cout << "k=" << std::setfill(' ') << std::setw(2) << k << ":";
        for (int d = (int)dist.size()-1; d >= 0; d--)
            for (int r : dist.at(d))
                std::cout << " " << std::setfill(' ') << std::setw(2) << r << "("<< std::setfill(' ') << std::setw(2) << d << ")";
        std::cout << std::endl;
    }
}


/**
 * @brief Solution::looping
 * @return True if the solutions is looping.
 * We compare the last step and two steps before. If they are the same, return true.
 * I use this in some algorithms that I know that usually oscillate between configurations.
 * However, there may be loops with different period lengths.
 */
bool Solution::looping() const
{
    if (makespan() < 2)
        return false;
    const int k1 = makespan();
    const int k2 = k1 - 2;
    for (int r = 0; r < m_ins.nb_robots(); ++r)
    {
        if (point(r, k1) != point(r, k2))
            return false;
    }
    std::cerr << "The solution repeats a configuration" << std::endl;
    return true;
}

/**
 * @brief Solution::configurationstep in a solution
 * @param k a step in the solution. If negative, it is set to the last step
 * @return A vector with the positions of robots at time `k`
 */
std::vector<Point> Solution::configuration(int k) const
{
    if (k < 0) k = makespan();
    std::vector<Point> config(m_ins.nb_robots());
    for (std::size_t r = 0; r < config.size(); ++r)
    {
        config.at(r) = point(r, k);
    }
    return config;
}

/**
 * @brief Solution::point
 * @return The position of the r-th robot at time `k`
 */
Point Solution::point(int r, int k) const
{
    assert(0 <= r && r < m_ins.nb_robots());
    assert(0 <= k && k < (int)m_steps.at(r).size());
    return m_steps.at(r).at(k);
}


/**
 * @brief Solution::makespan
 * @return the makespan (number of steps) of the solution
 * @note This assumes that each robot has the same number of steps
 * (even if it does not move)
 */
int Solution::makespan() const
{
    return nb_steps(0) - 1;
}


/**
 * @brief Solution::distance
 * @return the total number of moves of the robots
 */
int Solution::distance() const
{
    int sum = 0;
    for (int k = 0; k <= makespan()-1; ++k)
    {
        for (int r = 0; r < m_ins.nb_robots(); r++)
        {
            const Point &p = m_steps.at(r).at(k);
            const Point &q = m_steps.at(r).at(k+1);
            if (p != q)
                sum++;
        }
    }
    return sum;
}


/**
 * @brief Solution::bounding_box
 * @return Bounding box of the solution
 */
Box Solution::bounding_box() const
{
    int minx = point(0, 0).x, maxx(minx);
    int miny = point(0, 0).y, maxy(miny);
    int minz = 0, maxz = makespan();
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int k = minz; k <= maxz; k++)
        {
            const Point &p = point(r, k);
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }
    }
    return Box(Point(minx, miny), minz, Point(maxx, maxy), maxz);
}

void Solution::write_old(std::string comment, std::string filename) const
{
    std::cout << "-- Writing solution to " << filename << "..." << std::endl;
    std::ofstream file(filename.c_str(), std::ifstream::out | std::ifstream::binary);

    file << "{" << std::endl;
    file << "\t\"type\": \"Solution\"," << std::endl;
    file << "\t\"instance_name\": \"" << m_ins.name() << "\"," << std::endl;

    file << "\t\"meta\": {" << std::endl;
    file << "\t\t\"author\": \"Aldo Gonzalez-Lorenzo\"," << std::endl;
    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    struct std::tm * ptm = std::localtime(&tt);
    file << "\t\t\"date\": \"" << std::put_time(ptm,"%F %T") << "\"," << std::endl;
    file << "\t\t\"comment\": \"" << comment << "\"," << std::endl;
    file << "\t\t\"makespan\": " << makespan() << "," << std::endl;
    file << "\t\t\"distance\": " << distance() << "" << std::endl;
    file << "\t}," << std::endl;

    file << "\t\"steps\": [" << std::endl;
    const int nb_steps = m_steps.front().size();
    for (int k = 0; k < nb_steps; ++k)
    {
        file << "\t\t[" << std::endl;
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            file << "\t\t\t{\"id\": " << r << ", \"x\": " << m_steps.at(r).at(k).x << ", \"y\": " << m_steps.at(r).at(k).y << "}";
            if (r < m_ins.nb_robots()-1) file << ",";
            file << std::endl;
        }
        file << "\t\t]";
        if (k < nb_steps - 1) file << ",";
        file << std::endl;
    }
    file << "\t]," << std::endl;

    file << "\t\"obstacles\": [" << std::endl;
    for (int i = 0; i < m_ins.nb_obstacles(); i++)
    {
        const Point &p = m_ins.obstacle(i);
        file << "\t\t{\"x\": " << p.x << ", \"y\": " << p.y << "}";
        if (i < m_ins.nb_obstacles()-1) file << ",";
        file << std::endl;
    }
    file << "\t]" << std::endl;

    file << "}" << std::endl;
    file.close();
}

void Solution::write_steps(std::string comment, std::string filename) const
{
    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    struct std::tm * ptm = std::localtime(&tt);

    if (filename.empty())
    {
        std::ostringstream oss;
        if (m_ins.name().empty()) oss << "unnamed";
        else                      oss << m_ins.name();
        oss << ".agl." << std::put_time(ptm, "%Y%m%d-%H%M%S") << ".steps.json";
        filename = oss.str();
    }

    std::cout << "-- Writing steps to " << filename << "..." << std::endl;
    std::ofstream file(filename.c_str(), std::ifstream::out | std::ifstream::binary);

    file << "{" << std::endl;
    file << "\t\"instance\": \"" << m_ins.name() << "\"," << std::endl;

    file << "\t\"meta\": {" << std::endl;
    file << "\t\t\"checkpoints\": true, " << std::endl;
    file << "\t\t\"author\": \"Aldo Gonzalez-Lorenzo\"," << std::endl;
    file << "\t\t\"date\": \"" << std::put_time(ptm,"%F %T") << "\"," << std::endl;
    char hn[80]; gethostname(hn, 80);
    file << "\t\t\"host\": \"" << std::string(hn) << "\"," << std::endl;
    file << "\t\t\"computation_sec\": " << m_ins.elapsed_sec() << "," << std::endl;
    file << "\t\t\"comment\": \"" << comment << "\"" << std::endl;
    file << "\t}," << std::endl;

    file << "\t\"steps\": [" << std::endl;
    const int nb_steps = m_steps.front().size();
    for (int k = 0; k < nb_steps; ++k)
    {
        file << "\t\t[";
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            file << "[" << m_steps.at(r).at(k).x << ", " << m_steps.at(r).at(k).y << "]";
            if (r < m_ins.nb_robots()-1) file << ", ";
        }
        file << "]";
        if (k < nb_steps - 1) file << ", ";
        file << std::endl;
    }
    file << "\t]" << std::endl;

    file << "}" << std::endl;
    file.close();
}


/**
 * @brief Solution::add_missing_steps
 * Make all robots have the same number of steps by adding the last point several times
 */
void Solution::add_missing_steps()
{
    int max_k = 0; // maximum number of steps of a robot
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        max_k = std::max(max_k, (int)m_steps[r].size());
    }
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point p =  (m_steps.at(r).empty()) ? m_ins.s(r) : m_steps.at(r).back();
        m_steps.at(r).resize(max_k, p);
    }
}
