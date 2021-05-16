#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <string>
#include "instance.h"


class Solution
{
public:
    Solution(const Instance &ins);
    Solution(const Instance &ins, const std::string &filename);

    void read(const std::string &filename);
    void set_steps(const std::vector<std::vector<Point>> &steps);
    void clear_steps();
    void clear_steps(int r);

    void add_robot_position(int r, Point p);
    void set_robot_position(int r, int k, Point p);
    void write(std::string comment = "", std::string filename = "", bool do_check = true) const;
    void write_old(std::string comment = "", std::string filename = "") const;
    void write_steps(std::string comment = "", std::string filename = "") const;
    void print() const;

    void reduce_makespan();
    void reverse();
    void extend(const Solution &sol);

    bool check() const;
    bool solved() const;
    void print_race() const;

    bool looping() const;
    void add_missing_steps();
    inline int nb_steps(int r) const { return m_steps.at(r).size(); }

    inline Instance instance() const { return m_ins; }
    inline int nb_robots() const { return m_ins.nb_robots(); }
    inline int nb_obstacles() const { return m_ins.nb_obstacles(); }
    inline std::vector<std::vector<Point>> steps() const { return m_steps; }
    std::vector<Point> configuration(int k) const;
    Point point(int r, int k) const;
    int makespan() const;
    int distance() const;
    Box bounding_box() const;

private:
    std::vector<std::string> moves(int k) const;


private:
    const Instance &m_ins;
    std::vector<std::vector<Point>> m_steps; // for each robot r, m_steps[r] contains its sequence of steps
};

#endif // SOLUTION_H
