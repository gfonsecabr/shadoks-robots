#ifndef FINDERPOINTS_H
#define FINDERPOINTS_H

#include <ilcplex/ilocplex.h>

#include <map>

#include "instance.h"
#include "solution.h"

/**
 * @brief The FinderPoints class
 * Similar to @class SolverLP, this looks for a solution using CPLEX.
 * The difference is that we only look for a feasible solution, and we assume
 * that start and target positions are very close. Hence, we have much less
 * variables and constraints
 */
class FinderPoints
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
    FinderPoints(const Instance &ins, const Options &options);
    void set_k(int k);
    bool solve(int makespan);
    bool solve_sat(int makespan);
    inline Solution solution() const { return m_sol; }

private:
    void add_variables(IloModel model, IloNumVarArray x);
    void add_objective_function(IloModel model) const;
    void add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const;
    void constraint_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const;
    void constraint_uniqueness_splitted(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const;
    void constraint_start_target(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const;
    void constraint_continuity(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i, int j, int k) const;
    void constraint_coincidence(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const;
    void constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const;
    int index(int r, int i, int j, int k) const;
    inline bool is_var(int r, int i, int j, int k) const { return m_index.count(Tuple(r, i, j, k)) > 0; }
    void make_solution(IloNumArray vals);


    void make_variables();
    void make_clauses();
    void clause_uniqueness(int r, int k);
    void clause_start_target(int r);
    void clause_continuity(int r, int i, int j, int k);
    void clause_coincidence(int r1, int r2, int i, int j, int k);
    void clause_overlap(int r1, int r2, int i, int j, int k);
    void write_dimacs() const;

private:
    const Instance &m_ins;
    const Options &m_opt;
    Solution m_sol;
    int m_k; // number of steps fixed for solving the problem
    std::map<Tuple, int> m_index; // m_index[tup] = index of tup
    std::vector<Tuple> m_tuple;   // inverse of m_index
    std::vector<std::list<int>> m_cnf; // logical clauses
};

#endif // FINDERPOINTS_H
