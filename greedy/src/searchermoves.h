#ifndef SEARCHERMOVES_H
#define SEARCHERMOVES_H

#include <list>
#include <map>

#include <ilcplex/ilocplex.h>

#include "instance.h"
#include "solution.h"
#include "tools.h"

/**
 * @brief The SearcherMoves class
 * LP or SAT model for a searching problem (no predefined targets) for makespan.
 * I only implement LP path-based for the moment
 *
 * This is the move-based model, where I have variables for each robot-time-move
 * plus robot-end variables
 */
class SearcherMoves
{
public:
    SearcherMoves(const Instance &ins, const Options &options, int steps);
    void set_robot_position(int r, Point p);
    inline Point robot_position(int r, int k) const { return m_sol.point(r, k); }
    void set_weight(int r, Point p, double w);
    void set_weight(int r, int i, double w);
    inline int nb_endpoints() const { return m_point_with_index.size(); }
    void solve();
    inline Solution solution() const { return m_sol; }

private:
    void add_variables(IloModel model, IloNumVarArray x) const;
    void add_objective_function(IloModel model, IloNumVarArray x) const;
    void add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void constraint_endpoints(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_existence_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const;
    void constraint_volume(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const;
    void constraint_obstacle(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_bound(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i) const;
    void initial_solution(IloCplex& cplex, const IloNumVarArray& x) const;
    int index_of_xrkm(int r, int k, int m) const;
    int index_of_yrij(int r, Point p) const;
    void make_solution(IloNumArray vals);




private:
    const Instance &m_ins;
    const Options &m_opt;
    Solution m_sol;
    int m_k;
    std::vector<Point> m_point_with_index; // Functions between points (i,j) and integers
    std::map<Point,int> m_index_of_point;
    std::vector<Point> m_start; // start position of each robot in this subproblem
    std::vector<std::vector<double>> m_weights; // m_weights[r] is the vector with the weights of each of endpoints
};

#endif // SEARCHERMOVES_H
