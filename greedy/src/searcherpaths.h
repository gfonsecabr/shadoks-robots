#ifndef SEARCHERPATHS_H
#define SEARCHERPATHS_H

#include <list>
#include <map>

#include <ilcplex/ilocplex.h>

#include "instance.h"
#include "solution.h"
#include "tools.h"

/**
 * @brief The Searcher class
 * LP or SAT model for a searching problem (no predefined targets) for makespan.
 * I only implement LP path-based for the moment
 *
 * I think that I do not need the existence constraint because it is already implied
 * by the neighborhood constraint.
 * Also, I can maybe remove the  uniqueness constraint
 */
class SearcherPaths
{
public:
    SearcherPaths(const Instance &ins, const Options &options, int steps);
    void set_robot_position(int r, Point p);
    inline Point robot_position(int r, int k) const { return m_sol.point(r, k); }
    void set_weight(int r, Point p, double w);
    void set_weight(int r, int i, double w);
    inline int nb_endpoints() const { return m_point_with_index.size(); }
    std::vector<Point> path_with_index(int index) const;
    inline int nb_paths() const { return m_nb_paths; }
    void solve();
    void solve_bound();
    inline Solution solution() const { return m_sol; }
    void print_weights() const;

private:
    void make_paths();
    void precompute_coincidence();
    void precompute_overlap();
    int index_of_path(const std::list<Point> &moves) const;
    void add_variables(IloModel model, IloNumVarArray x) const;
    void add_variables_bound(IloModel model, IloNumVarArray x) const;
    void add_objective_function(IloModel model, IloNumVarArray x) const;
    void add_objective_function_bound(IloModel model, IloNumVarArray x) const;
    void add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void add_constraints_bound(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void constraint_existence_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_volume(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_obstacle(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i) const;
    void constraint_bound(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i) const;
    int index_of_xri(int r, int i) const;
    int index_of_b() const;
    void initial_solution(IloCplex& cplex, const IloNumVarArray& x) const;
    void make_solution(IloNumArray vals);
    void infeasibility(IloCplex &cplex, IloNumVarArray &var, IloRangeArray &con) const;

    Point endpoint(int i) const;


private:
    const Instance &m_ins;
    const Options &m_opt;
    Solution m_sol;
    int m_k;
    std::vector<Point> m_point_with_index; // Functions between points (i,j) and integers
    std::map<Point,int> m_index_of_point;
    int m_nb_paths; // this is just 5**m_k
    std::vector<std::vector<Point>> m_path_with_index;
    std::vector<Point> m_start; // start position of each robot in this subproblem
    std::vector<std::vector<double>> m_weights; // m_weights[r] is the vector with the weights of each endpoint
    std::map<Point, std::list<std::pair<int, int>>> m_coincidence;
    std::map<Point, std::list<std::pair<int, int>>> m_overlap;
};

#endif // SEARCHERPATHS_H
