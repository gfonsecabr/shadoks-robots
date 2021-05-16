#include "instance.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm> // std::reverse()

#include "include/rapidjson/document.h"
#include "include/rapidjson/istreamwrapper.h"
#include "include/rapidjson/writer.h"
#include "include/rapidjson/stringbuffer.h"
#include "include/rapidjson/ostreamwrapper.h"

Instance::Instance() :
    m_reversed(false)
{
}


Instance::Instance(std::string filename) :
    m_reversed(false)
{
    read(filename);
    std::clog << "Instance " << m_name << " loaded" << std::endl;
}


/**
 * @brief Instance::add_robot
 * @param start_x
 * @param start_y
 * @param end_x
 * @param end_y
 */
void Instance::add_robot(int start_x, int start_y, int target_x, int target_y)
{
    const Point start(start_x, start_y);
    const Point target(target_x, target_y);
    add_robot(start, target);
}


void Instance::add_robot(Point start, Point target)
{
    m_start.push_back(start);
    m_target.push_back(target);
    /// @todo check that there is not a collision between the new states, and also with the obstacles
}


void Instance::add_obstacle(int x, int y)
{
    /// @todo update the bounding box?
    /// I think that I should do it in the class that treats the instance
    const Point p(x, y);
    m_obstacles.push_back(p);
}



void Instance::read(std::string filename)
{
    std::ifstream in(filename.c_str(), std::ifstream::in | std::ifstream::binary);
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

    m_name = doc["name"].GetString();

    const rapidjson::Value& obstacles = doc["obstacles"];
    for (rapidjson::Value::ConstValueIterator it = obstacles.Begin(); it != obstacles.End(); ++it)
    {
        const std::size_t x = (*it)[0].GetInt();
        const std::size_t y = (*it)[1].GetInt();
        add_obstacle(x, y);
    }

    std::vector<int> start_x, start_y;
    const rapidjson::Value& start = doc["starts"];
    for (rapidjson::Value::ConstValueIterator itr = start.Begin(); itr != start.End(); ++itr)
    {
        const std::size_t x = (*itr)[0].GetInt();
        const std::size_t y = (*itr)[1].GetInt();
        start_x.push_back(x);
        start_y.push_back(y);
    }

    std::vector<int> target_x, target_y;
    const rapidjson::Value& target = doc["targets"];
    for (rapidjson::Value::ConstValueIterator itr = target.Begin(); itr != target.End(); ++itr)
    {
        const std::size_t x = (*itr)[0].GetInt();
        const std::size_t y = (*itr)[1].GetInt();
        target_x.push_back(x);
        target_y.push_back(y);
    }

    for (std::size_t i = 0; i < start_x.size(); ++i)
    {
       add_robot(start_x.at(i), start_y.at(i), target_x.at(i), target_y.at(i));
    }
}

/**
 * @brief Instance::set_start
 * @param configuration a vector, where configuration[r] is the position of the r-th robot
 * Set the start configuration from a configuration
 */
void Instance::set_start(const std::vector<Point> &configuration)
{
    assert(configuration.size() == m_start.size());
    m_start = configuration;
}


void Instance::set_target(const std::vector<Point> &configuration)
{
    assert(configuration.size() == m_target.size());
    m_target = configuration;
}


/**
 * @brief Instance::reverse
 * Exchange start and target positions
 */
void Instance::reverse()
{
    m_start.swap(m_target);
    m_reversed = !m_reversed;
}


void Instance::print() const
{
    for (int r = 0; r < nb_robots(); r++)
    {
        const std::string str_s = s(r).str();
        const std::string str_t = t(r).str();
        std::cout << "r = " << r << ": " << str_s << " -> " << str_t << std::endl;
    }
    std::cout << "Obstacles: ";
    for (const Point &ob : m_obstacles)
        std::cout << ob.str() << " ";
    std::cout << std::endl;
}


void Instance::write(std::string filename, std::string name) const
{
    std::cout << "-- Writing instance to " << filename << "..." << std::endl;
    std::ofstream file(filename.c_str(), std::ifstream::out | std::ifstream::binary);

    file << "{" << std::endl;
    file << "\t\"type\": \"Instance\"," << std::endl;
    file << "\t\"instance_name\": \"" << name << "\"," << std::endl;

    file << "\t\"meta\": {" << std::endl;
    file << "\t\t\"format\": \"G\"" << std::endl;
    file << "\t}," << std::endl;

    file << "\t\"start\": [" << std::endl;
    for (int r = 0; r < nb_robots(); r++)
    {
        file << "\t\t{\"id\": " << r << ", \"x\": " << s(r).x << ", \"y\": " << s(r).y << "}";
        if (r < nb_robots()-1) file << ",";
        file << std::endl;
    }
    file << "\t]," << std::endl;

    file << "\t\"target\": [" << std::endl;
    for (int r = 0; r < nb_robots(); r++)
    {
        file << "\t\t{\"id\": " << r << ", \"x\": " << t(r).x << ", \"y\": " << t(r).y << "}";
        if (r < nb_robots()-1) file << ",";
        file << std::endl;
    }
    file << "\t]," << std::endl;

    file << "\t\"obstacles\": [" << std::endl;
    for (int i = 0; i < nb_obstacles(); i++)
    {
        const Point &p = obstacle(i);
        file << "\t\t{\"id\": " << i << ", \"x\": " << p.x << ", \"y\": " << p.y << "}";
        if (i < nb_obstacles()-1) file << ",";
        file << std::endl;
    }
    file << "\t]" << std::endl;

    file << "}" << std::endl;
    file.close();
}




/**
 * @brief Instance::is_obstacle
 * @return True if there is an obstacle at point `p`.
 * If this is not fast enough, I could store obstacles in a set (the only problem is that I could not index them)
 * @note Complexity linear in the number of obstacles
 */
bool Instance::is_obstacle(const Point &p) const
{
    for (const Point &ob : m_obstacles)
        if (ob == p) return true;
    return false;
}


double Instance::elapsed_sec() const
{
    auto cur_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = cur_time - m_start_time;
    return elapsed_seconds.count();
}

/**
 * @brief Instance::write_distances
 * Compute the distance of each robot, sort them in descending order and write it
 * in a JSON file
 */
void Instance::write_distances() const
{
    std::vector<std::list<int>> dist(250); // this is enough for the challenge instances
    for (int r = 0; r < nb_robots(); r++)
    {
        const int d_r = distance(r);
        dist.at(d_r).push_back(r);
    }

    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);
    writer.StartObject();
    writer.Key("instance");
    writer.String(m_name.c_str());
    writer.Key("distance");
    writer.StartObject();
    for (int i = (int)dist.size()-1; i >= 0; --i)
    {
        if (!dist.at(i).empty())
        {
            writer.Key(std::to_string(i).c_str());
            writer.StartArray();
            for (int r : dist.at(i))
                writer.Uint(r);
            writer.EndArray();
        }
    }
    writer.EndObject();
    writer.EndObject();

    const std::string filename = m_name + ".dist.json";
    std::ofstream file(filename.c_str(), std::ifstream::out | std::ifstream::binary);
    file << s.GetString();
    file.close();
}


/**
 * @brief Instance::distance
 * @return minimum length of a path from the start to the target of the r-th robot.
 * @todo I have to consider the case where there are obstacles and compute the true distance between starts and targets.
 */
int Instance::distance(int r) const
{
    std::vector<Point> unused_path;
    const int dist = a_star_search(r, unused_path);
    return dist;
}

/**
 * @brief Instance::distance
 * @return Distance between two points
 */
int Instance::distance(Point p1, Point p2) const
{
    std::vector<Point> unused_path;
    const int dist = a_star_search(p1, p2, unused_path);
    return dist;
}


int Instance::distance_bound() const
{
    int sum = 0;
    for (int r = 0; r < nb_robots(); r++)
        sum += distance(r);
    return sum;
}


int Instance::makespan_bound() const
{
    int max_dist = 0;
    for (int r = 0; r < nb_robots(); r++)
        max_dist = std::max(max_dist, distance(r));
    return max_dist;
}


/**
 * @brief Instance::bounding_box_start
 * @return The bounding box of the start points
 */
Box Instance::bounding_box_start() const
{
    int minx = m_start.at(0).x, maxx(minx);
    int miny = m_start.at(0).y, maxy(miny);
    for (auto it = m_start.cbegin(); it != m_start.cend(); ++it)
    {
        minx = std::min(minx, it->x);
        maxx = std::max(maxx, it->x);
        miny = std::min(miny, it->y);
        maxy = std::max(maxy, it->y);
    }
    return Box(Point(minx, miny), 0, Point(maxx, maxy), 0);
}

/**
 * @brief Instance::bounding_box
 * @return The bounding box of the points in the start configuration, plus a padding
 * @note I use the padding for the A* search, which needs one more pixel in each side to go around the obstacles.
 */
Box Instance::bounding_box(int padding, std::vector<Point> extra_points) const
{
    int minx = m_start.at(0).x, maxx(minx);
    int miny = m_start.at(0).y, maxy(miny);
    for (auto it = m_start.cbegin(); it != m_start.cend(); ++it)
    {
        minx = std::min(minx, it->x);
        maxx = std::max(maxx, it->x);
        miny = std::min(miny, it->y);
        maxy = std::max(maxy, it->y);
    }
    for (auto it = m_target.cbegin(); it != m_target.cend(); ++it)
    {
        minx = std::min(minx, it->x);
        maxx = std::max(maxx, it->x);
        miny = std::min(miny, it->y);
        maxy = std::max(maxy, it->y);
    }
    for (auto it = m_obstacles.cbegin(); it != m_obstacles.cend(); ++it)
    {
        minx = std::min(minx, it->x);
        maxx = std::max(maxx, it->x);
        miny = std::min(miny, it->y);
        maxy = std::max(maxy, it->y);
    }
    for (auto it = extra_points.cbegin(); it != extra_points.cend(); ++it)
    {
        minx = std::min(minx, it->x);
        maxx = std::max(maxx, it->x);
        miny = std::min(miny, it->y);
        maxy = std::max(maxy, it->y);
    }
    minx -= padding;
    maxx += padding;
    miny -= padding;
    maxy += padding;
    return Box(Point(minx, miny), 0, Point(maxx, maxy), 0);
}

struct Node
{
    Point p;
    int f;

    Node (Point _p, int _f) : p(_p), f(_f) {}
    bool operator <(const Node& n) const { return f > n.f; }
};



/**
 * @brief Instance::a_star_search
 * Search a path from the start to the target of the r-th robot
 * using the A* search algorithm.
 * Sources for the code:
 * - https://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/
 * - https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
 *
 * If there is no path because the target is an obstacle, this returns an empty path.
 * @todo What if there is no path because it is impossible to reach the target?
 * @return the distance to the point. -1 if no path.
 */
int Instance::a_star_search(const Point &start, const Point &target, std::vector<Point> &path,
                            const std::set<Point> &extra_obstacles) const
{
    path.clear();
    if (is_obstacle(start) || extra_obstacles.count(start) > 0)
        return -1;
    if (is_obstacle(target) || extra_obstacles.count(target) > 0)
        return -1;
    if (m_obstacles.empty() && extra_obstacles.empty()) // if no obstacles, this is just the L1 distance
    {
        Point p = start;
        path.push_back(p);
        while (std::abs(p.x - target.x) > 0)
        {
            p.x += (p.x < target.x) ? 1 : -1;
            path.push_back(p);
        }
        while (std::abs(p.y - target.y) > 0)
        {
            p.y += (p.y < target.y) ? 1 : -1;
            path.push_back(p);
        }
        return start.dist(target);
    }

    std::priority_queue<Node> openSet;
    openSet.push(Node(start, start.dist(target)));

    std::set<Point> closedSet;

    std::map<Point, Point> cameFrom;

    std::map<Point, int> g; // g = 0 by default
    g[start] = 0;

    std::vector<Point> extra_points{start, target}; // @note what is this?
    const Box bbox = bounding_box(1, extra_points); // bounding box of the instance with a padding

    while (!openSet.empty())
    {
        Node current = openSet.top();
        if (current.p == target)
        {
            Point p = current.p;
            path.push_back(p);
            while (cameFrom.count(p) > 0)
            {
                p = cameFrom.at(p);
                path.push_back(p);
            }
            std::reverse(path.begin(), path.end());
            return path.size()-1;
        }

        openSet.pop();
        closedSet.insert(current.p);

        for (int i = 0; i < 4; i++)
        {
            const Point neighbor = current.p.neighbor(i);
            if (!is_obstacle(neighbor) && extra_obstacles.count(neighbor) == 0
                    && bbox.inside(neighbor) && closedSet.count(neighbor) == 0)
            {
                const int tentative_g = g.at(current.p) + 1;
                if (g.count(neighbor) == 0 || tentative_g < g.at(neighbor))
                {
                    cameFrom[neighbor] = current.p;
                    g[neighbor] = tentative_g;
                    int f = tentative_g + neighbor.dist(target);
                    openSet.push(Node(neighbor, f)); /// @todo I am not sure about this because the same point can be several times here
                }
            }
        }
    }
    return -1;
}



int Instance::a_star_search(int r, std::vector<Point> &path,
                            const std::set<Point> &extra_obstacles) const
{
    assert(0 <= r && r < nb_robots());
    const Point start = s(r);
    const Point target = t(r);
    return a_star_search(start, target, path, extra_obstacles);
}


/**
 * @brief Instance::dijkstra
 * I use a list of extra obstacles to allow repetitions.
 */
int Instance::dijkstra(int r, std::vector<Point> &path, const std::list<Point> &extra_obstacles) const
{
    path.clear();
    const int padding = 3; // @todo Not very elegant
    std::set<Point> frontier;
    std::map<Point, double> dist;
    std::map<Point, Point> prev;
    const Box bbox = bounding_box(padding); // bounding box of the instance with a padding
    const double penalty = 1.0 / nb_robots();

    frontier.insert(s(r));
    dist[s(r)] = 0;
    while (!frontier.empty())
    {
        Point closest;
        double min_dist = 1000000;
        for (const Point &p : frontier)
            if (dist.count(p) > 0 && dist.at(p) < min_dist)
            {
                closest = p;
                min_dist = dist.at(p);
            }
        frontier.erase(frontier.find(closest));

        if (closest == t(r))
        {
            Point p = closest;
            path.push_back(p);
            while (prev.count(p) > 0)
            {
                p = prev.at(p);
                path.push_back(p);
            }
            std::reverse(path.begin(), path.end());
            return path.size()-1;
        }

        for (int i = 0; i < 4; i++)
        {
            const Point neighbor = closest.neighbor(i);
            if (!is_obstacle(neighbor) && bbox.inside(neighbor))
            {
                double edge_weight = 1;
                for (const Point ob : extra_obstacles)
                {
                    if (ob == closest)
                        edge_weight += penalty;
                    if (ob == neighbor)
                        edge_weight += penalty;
                }
                if (dist.count(neighbor) == 0 || dist.at(closest) + edge_weight <  dist.at(neighbor))
                {
                    frontier.insert(neighbor);
                    dist[neighbor] = dist.at(closest) + edge_weight;
                    prev[neighbor] = closest;
                }
            }
        }

    }
    std::clog << "Target not found, dijsktra returns an empty path" << std::endl;
    return -1;
}


void Instance::make_distance_map(Point p, const Box &bbox, std::map<Point, int> &dist_map) const
{
    dist_map.clear();
    assert(bbox.inside(p));

    std::queue<Point> fifo;
    fifo.push(p);
    dist_map[p] = 0;
    while (!fifo.empty())
    {
        const Point cur_point = fifo.front(); fifo.pop();
        const int cur_dist = dist_map.at(cur_point);
        for (int i = 0; i < 4; i++)
        {
            const Point neighbor = cur_point.neighbor(i);
            if (!is_obstacle(neighbor) && bbox.inside(neighbor))
            {
                if (dist_map.count(neighbor) == 0)
                {
                    fifo.push(neighbor);
                    dist_map[neighbor] = cur_dist + 1;
                }
            }
        }
    }
}

/**
 * @brief Instance::depth
 * Signed distance of the point `p` in the bounding box `bbox`.
 * - If the point is inside the bounding box, this is the distance to any point
 *   outside the bounding box
 * - If the point is outside, this is the distance to the bounding box with a
 *   a negative sign
 */
int Instance::depth(Point p, const Box &bbox) const
{
    if (!bbox.inside(p))
    {
        const int dist_x = std::min(p.x - bbox.p1.x, bbox.p2.x - p.x);
        const int dist_y = std::min(p.y - bbox.p1.y, bbox.p2.y - p.y);
        return std::min(dist_x, dist_y) + 1; // +1 because there are no points at distance 0
    }
    /* if the points is inside */
    std::queue<Point> fifo;
    fifo.push(p);
    std::map<Point, int> dist_map;
    dist_map[p] = 0;
    while (!fifo.empty())
    {
        const Point cur_point = fifo.front(); fifo.pop();
        const int cur_dist = dist_map.at(cur_point);
        // if we are outside the bounding box, return this distance
        if (!bbox.inside(cur_point))
            return cur_dist;
        for (int i = 0; i < 4; i++)
        {
            const Point neighbor = cur_point.neighbor(i);
            if (!is_obstacle(neighbor))
            {
                if (dist_map.count(neighbor) == 0)
                {
                    fifo.push(neighbor);
                    dist_map[neighbor] = cur_dist + 1;
                }
            }
        }
    }
    /* if the point is not connected to the outside, return + infinity */
    return std::numeric_limits<int>::max();
}
