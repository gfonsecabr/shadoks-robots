#ifndef SEARCHERBOUNDEDPATHS_H
#define SEARCHERBOUNDEDPATHS_H

#include <list>
#include <map>
#include <ilcplex/ilocplex.h>
#include "instance.h"
#include "solution.h"
#include "tools.h"

/**
 * @brief The SearcherBoundedPaths class
 * Model for the divided greedy solver.
 */
class SearcherBoundedPaths
{
    struct Tuple
    {
        int r, i;
        Tuple() : r(0), i(0) {}
        Tuple (int _r, int _i) : r(_r), i(_i) {}
        bool operator <(const Tuple& t) const
        {
            return (r < t.r) || (r == t.r && i < t.i);
        }
    };

public:
    SearcherBoundedPaths(const Instance &ins, const Solution &sol, const std::vector<bool> planned, int steps, const Options &options);
    inline void set_start(const std::vector<Point> &start) { m_start = start; }
    inline void set_bbox(const Box &bbox) { m_bbox = bbox; }
    inline void set_flap(int flap) { m_flap = flap; }
    void set_robot_position(int r, Point p);
    void set_planned(int r);
    inline Point robot_position(int r, int k) const { return m_sol.point(r, k); }
    void set_weight(int r, Point p, double w);
    void set_weight(int r, int i, double w);
    inline int nb_endpoints() const { return m_point_with_index.size(); }
    std::vector<Point> path_with_index(int index) const;
    inline int nb_paths() const { return m_nb_paths; }
    bool solve();
    inline Solution solution() const { return m_sol; }
    void clear_model();
    void print_weights() const;


private:
    void make_paths();
    void precompute_coincidence();
    void precompute_overlap();
    int index_of_path(const std::list<Point> &moves) const;
    int index_of_path(int r) const;
    void add_variables(IloModel model, IloNumVarArray x);
    void add_objective_function(IloModel model, IloNumVarArray x) const;
    void add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void constraint_existence_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_volume(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_obstacle(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i) const;
    inline bool is_var(int r, int i) const { return m_index.count(Tuple(r, i)) > 0; }
    int index_of_xri(int r, int i) const;
    void initial_solution(IloCplex& cplex, const IloNumVarArray& x) const;
    void make_solution(IloNumArray vals);
    void infeasibility(IloCplex &cplex, IloNumVarArray &var, IloRangeArray &con) const;
    Point endpoint(int i) const;
    bool point_in_box_with_flaps(Point p) const;
    bool point_in_flap(Point p) const;
    bool path_in_box_with_flaps(int r, int i) const;
    bool path_in_flap(int r, int i) const;

private:
    const Instance &m_ins;
    Box m_bbox;
    Solution m_sol;
    std::vector<bool> m_planned; // m_planned[r] == true if the robot is already planned
    int m_k;
    int m_flap; // we add variables for the paths contained in m_bbox on it its flaps with size m_flap
    const Options &m_opt;
    std::map<Tuple, int> m_index; // m_index[tup] = index of tup
    std::vector<Tuple> m_tuple;   // inverse of m_index

    std::vector<Point> m_point_with_index; // Functions between points (i,j) and integers
    std::map<Point,int> m_index_of_point;
    int m_nb_paths; // this is just 5**m_k
    std::vector<std::vector<Point>> m_path_with_index;
    std::vector<Point> m_start; // start position of each robot in this subproblem
    std::vector<std::vector<double>> m_weights; // m_weights[r] is the vector with the weights of each endpoint
    std::map<Point, std::list<std::pair<int, int>>> m_coincidence;
    std::map<Point, std::list<std::pair<int, int>>> m_overlap;
};

#endif // SEARCHERBOUNDEDPATHS_H
