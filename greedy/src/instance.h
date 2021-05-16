#ifndef INSTANCE_H
#define INSTANCE_H

#include "tools.h"

#include <chrono>
#include <string>
#include <vector>
#include <list>
#include <set>
#include <map>

/**
 * @brief The PlaneGraph class
 */
class Instance
{
public:
    Instance();
    Instance(std::string filename);
    void add_robot(int start_x, int start_y, int target_x, int target_y);
    void add_robot(Point start, Point target);
    void add_obstacle(int x, int y);
    inline bool is_reversed() const { return m_reversed; }
    inline bool has_obstacles() const { return m_obstacles.size() > 0; }


    void set_start(const std::vector<Point> &configuration);
    void set_target(const std::vector<Point> &configuration);
    void reverse();
    void print() const;
    void write(std::string filename, std::string name = "") const;

    inline int nb_robots() const { return m_start.size(); }
    inline Point s(int r) const { return m_start.at(r); }
    inline Point t(int r) const { return m_target.at(r); }
    inline int nb_obstacles() const { return m_obstacles.size(); }
    inline Point obstacle(int i) const { return m_obstacles.at(i); }
    inline std::string name() const { return m_name; }
    bool is_obstacle(const Point &p) const;
    double elapsed_sec() const;
    void write_distances() const;

    int distance(int r) const;
    int distance(Point p1, Point p2) const;
    int distance_bound() const;
    int makespan_bound() const;
    void make_distance_map(Point p, const Box &bbox, std::map<Point, int> &dist_map) const;
    Box bounding_box_start() const;
    Box bounding_box(int padding, std::vector<Point> extra_points = std::vector<Point>()) const;

    int a_star_search(int r, std::vector<Point> &path,
                      const std::set<Point> &extra_obstacles = std::set<Point>()) const;
    int a_star_search(const Point &start, const Point &target, std::vector<Point> &path,
                      const std::set<Point> &extra_obstacles = std::set<Point>()) const;
    int dijkstra(int r, std::vector<Point> &path, const std::list<Point> &extra_obstacles = std::list<Point>()) const;
    int depth(Point p, const Box &bbox) const;

private:
    void read(std::string filename);


private:
    std::string m_name;
    std::vector<Point> m_start;
    std::vector<Point> m_target;
    std::vector<Point> m_obstacles;

    std::string m_comment;
    const std::chrono::steady_clock::time_point m_start_time = std::chrono::steady_clock::now();
    bool m_reversed;
};

#endif // INSTANCE_H
