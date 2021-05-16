#ifndef SEARCHERPOINTS_H
#define SEARCHERPOINTS_H

//#include <list>
#include <map>

#include <ilcplex/ilocplex.h>

#include "instance.h"
#include "solution.h"
#include "tools.h"


/**
 * @brief The SearcherPoints class
 * LP or SAT model for a searching problem (no predefined targets) for makespan.
 * I only implement LP point-based for the moment
 *
 * This is the point-based model, where I have variables for each robot-space-time
 */
class SearcherPoints
{
    struct Tuple
    {
        int r, i, j, k;
        Tuple() : r(0), i(0), j(0), k(0) {}
        Tuple (int _r, int _i, int _j, int _k) : r(_r), i(_i), j(_j), k(_k) {}
        bool operator <(const Tuple& t) const
        {
            return (r < t.r)
                    || (r == t.r && i < t.i)
                    || (r == t.r && i == t.i && j < t.j)
                    || (r == t.r && i == t.i && j == t.j && k < t.k);
        }
    };

public:
    SearcherPoints(const Instance &ins, const Options &options, int steps);
    void set_robot_position(int r, Point p);
    inline Point robot_position(int r, int k) const { return m_sol.point(r, k); }
    void set_weight(int r, Point p, double w);
    void set_weight(int r, int i, double w);
    inline int nb_endpoints() const { return m_point_with_index.size(); }
    void solve();
    inline Solution solution() const { return m_sol; }

private:
    void add_variables(IloModel model, IloNumVarArray x);
    void add_objective_function(IloModel model, IloNumVarArray x) const;
    void add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void constraint_start(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const;
    void constraint_uniqueness_splitted(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const;
    void constraint_continuity(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i, int j, int k) const;
    void constraint_coincidence(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const;
    void constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const;
    void initial_solution(IloCplex& cplex, const IloNumVarArray& x) const;
    void make_solution(IloNumArray vals);

    inline bool is_var(int r, int i, int j, int k) const { return m_index.count(Tuple(r, i, j, k)) > 0; }
    int index_of_xrijk(int r, int i, int j, int k) const;

private:
    const Instance &m_ins;
    const Options &m_opt;
    Solution m_sol;
    int m_k;
    std::vector<Point> m_start; // start position of each robot in this subproblem
    std::vector<std::vector<double>> m_weights; // m_weights[r] is the vector with the weights of each endpoint
    std::vector<Point> m_point_with_index; // Functions between points (i,j) and integers
    std::map<Point,int> m_index_of_point;
    std::map<Tuple, int> m_index; // m_index[tup] = index of tup
    std::vector<Tuple> m_tuple;   // inverse of m_index

};

#endif // SEARCHERPOINTS_H
