#include "finderpoints.h"

FinderPoints::FinderPoints(const Instance &ins, const Options &options) :
    m_ins(ins), m_opt(options), m_sol(ins)
{
}


void FinderPoints::set_k(int k)
{
    assert(k >= 0);
    m_k = k;
}


/**
 * @brief FinderPoints::solve
 * Make the optimization problem and solve it.
 * You should not optimize makespan and distance at the same time
 */
bool FinderPoints::solve(int makespan)
{
    if (makespan < m_ins.makespan_bound())
        return false;

    std::clog << "Solving with FinderPoints::solve(makespan=" << makespan << ")..." << std::endl;

    /* reset */
    m_sol.clear_steps();
    set_k(makespan);
    m_index.clear();
    m_tuple.clear();

    IloEnv env;
    try {
        /* Build the model */
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);
        add_variables(model, var);
        add_objective_function(model);
        add_constraints(model, var, con);
        std::cout << m_ins.nb_robots() << " robots, " << m_k+1 << " steps." << std::endl;
        std::cout << var.getSize() << " variables, " << con.getSize() << " constraints." << std::endl;
//        std::cout << con << std::endl;

        /* Solve the model */
        IloCplex cplex(model);
        if (m_opt.time_limit >= 0)
            cplex.setParam(IloCplex::Param::TimeLimit, m_opt.time_limit); // Bound the time spent on solving the problem
//        cplex.setParam(IloCplex::Param::Parallel, 1); // Deterministic approach, always the same solution
//        cplex.setOut(env.getNullStream());  // don't output stuff
//        cplex.setParam(IloCplex::Param::MIP::Display, 1); // Print few things
//        cplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0); // I am not sure that this is used
//        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap   , 0);

        // Optimize the problem and obtain solution.
        if ( !cplex.solve() ) {
            std::cout << cplex.getStatus() << std::endl;
            return false;
        }

        /* Get the solution */
        IloNumArray vals(env);
        cplex.getValues(vals, var);
        make_solution(vals);
//        m_sol.reduce_makespan();
//        std::cout << "min makespan: " << m_ins.makespan_bound() << ", min distance: " << m_ins.distance_bound() << std::endl;
//        std::cout << "Makespan: " << m_sol.makespan() << ", distance: " << m_sol.distance() << std::endl;
//        m_sol.print();

        vals.end();
        con.end();
        var.end();
    }
    catch (IloException& e) {
        std::cerr << "Concert exception caught: " << e << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
    }

    env.end();
    return true;
}


/**
 * @brief FinderPoints::add_variables
 * Make the variables and add them to the model
 * Add variables only if:
 * - they are close enough to the start
 * - they are close enough to the target
 * - there is no obstacle
 * Also, make the indices for the variables.
 */
void FinderPoints::add_variables(IloModel model, IloNumVarArray x)
{
    IloEnv env = model.getEnv();

    // add the boolean variables x_{r,i,j,k}
    int index = 0;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_ins.s(r);
        const Point t = m_ins.t(r);
        for (int k = 0; k <= m_k; k++)
        {
            for (int i = s.x-k; i <= s.x+k; i++)
            {
                for (int j = s.y-k; j <= s.y+k; j++)
                {
                    const Point p(i, j);
                    if (p.dist(s) <= k && p.dist(t) <= m_k-k && !m_ins.is_obstacle(p))
                    {
                        const Tuple tup(r, i, j, k);
                        assert(m_index.count(tup) == 0);
                        m_index[tup] = index;
                        m_tuple.push_back(tup);
                        index++;
                        const std::string label = "x_r" + std::to_string(r) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k);
                        x.add(IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, label.c_str()));
                    }
                }
            }
        }
    }
}

/**
 * @brief FinderPoints::add_objective_function
 * The objective function is zero, we just look for a feasible solution.
 */
void FinderPoints::add_objective_function(IloModel model) const
{
    IloEnv env = model.getEnv();
    IloExpr obj_expr(env);
    IloObjective obj = IloMinimize(env, obj_expr);
    obj_expr.end();
    model.add(obj);
}


void FinderPoints::add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_ins.s(r);
        constraint_start_target(model, x, con, r);
        for (int k = 0; k <= m_k; k++)
        {
            constraint_uniqueness(model, x, con, r, k);
//            constraint_uniqueness_splitted(model, x, con, r, k);
            for (int i = s.x-k; i <= s.x+k; i++)
            {
                for (int j = s.y-k; j <= s.y+k; j++)
                {
                    constraint_continuity(model, x, con, r, i, j, k);
                    for (int r2 = 0; r2 < m_ins.nb_robots(); r2++)
                    {
                        constraint_coincidence(model, x, con, r, r2, i, j, k);
                        constraint_overlap(model, x, con, r, r2, i, j, k);
                    }
                }
            }
        }
    }
    model.add(con);
}

/**
 * Add constraints that ensure that for each robot and each time step, it occupies a single position
 * @note I do not need the existence part (given by start + neighborhood) and I can split the uniqueness
 * @todo Is there any difference in setting the lower bound?
 */
void FinderPoints::constraint_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const
{
    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    const Point &s = m_ins.s(r);
    for (int i = s.x-k; i <= s.x+k; ++i)
    {
        for (int j = s.y-k; j <= s.y+k; ++j)
        {
            if (is_var(r,i,j,k))
                c_expr += x[index(r,i,j,k)];
        }
    }
    const std::string label = "cu_r" + std::to_string(r) + "_k" + std::to_string(k);
//    IloRange constraint(env, 1.0, c_expr, 1.0, label.c_str()); // existence, too, already implied by continuity
    IloRange constraint(env, -IloInfinity, c_expr, 1.0, label.c_str());
    con.add(constraint);
    c_expr.end();
}

/**
 * @brief SearcherPoints::constraint_uniqueness_splitted
 * Similar to SearcherPoints::constraint_uniqueness_splitted, but it splits the
 * constraint into many pairwise constraints.
 * @todo Is this faster?
 */
void FinderPoints::constraint_uniqueness_splitted(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const
{
    IloEnv env = model.getEnv();
    const std::string label = "cu_r" + std::to_string(r) + "_k" + std::to_string(k);
    const Point &s = m_ins.s(r);
    std::vector<std::pair<int, int>> pairs;
    for (int i = s.x-k; i <= s.x+k; ++i)
        for (int j = s.y-k; j <= s.y+k; ++j)
            if (is_var(r, i, j ,k))
                pairs.push_back(std::pair<int, int>(i, j));
    for (const auto &pair1 : pairs)
        for (const auto &pair2 : pairs)
            if (index(r, pair1.first, pair1.second, k) < index(r, pair2.first, pair2.second, k))
            {
                IloExpr c_expr(env);
                c_expr += x[index(r, pair1.first, pair1.second, k)];
                c_expr += x[index(r, pair2.first, pair2.second, k)];
                IloRange constraint(env, -IloInfinity, c_expr, 1.0, label.c_str());
                con.add(constraint);
                c_expr.end();
            }
}

/**
 * The variables at the first and the last steps must be true.
 * @note They are the only variables at those steps, so this implies uniqueness.
 */
void FinderPoints::constraint_start_target(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const
{
    IloEnv env = model.getEnv();

    const std::string label_s = "cs_r" + std::to_string(r);
    const Point s = m_ins.s(r);
    IloRange constraint_s(env, 1.0, x[index(r, s.x, s.y, 0)], 1.0, label_s.c_str());
    con.add(constraint_s);

    const std::string label_t = "ct_r" + std::to_string(r);
    const Point t = m_ins.t(r);
    IloRange constraint_t(env, 1.0, x[index(r, t.x, t.y, m_k)], 1.0, label_t.c_str());
    con.add(constraint_t);
}

/**
 * @brief FinderPoints::constraint_continuity
 * Add the contraint that ensures that at each time, each robot stays in its 4-neighborhood.
 * If a robot is in a given position, at the next step it must be in some of the 5 possible positions
 * The constraint is: x - y1 ... -y5 <= 0
 * Hence, if the robot is there, the same robot must also be in any of the 5 places
 */
void FinderPoints::constraint_continuity(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i, int j, int k) const
{
    if (!(k+1 <= m_k))
        return;

    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    if (is_var(r, i, j, k)) // initial position
        c_expr += x[index(r, i, j, k)];
    else
        return; // only add constraint if that variable exist
    if (is_var(r,i  ,j  ,k+1)) c_expr -= x[index(r, i  , j  , k+1)]; // same at next step
    if (is_var(r,i-1,j  ,k+1)) c_expr -= x[index(r, i-1, j  , k+1)]; // west
    if (is_var(r,i+1,j  ,k+1)) c_expr -= x[index(r, i+1, j  , k+1)]; // east
    if (is_var(r,i  ,j-1,k+1)) c_expr -= x[index(r, i  , j-1, k+1)]; // south
    if (is_var(r,i  ,j+1,k+1)) c_expr -= x[index(r, i  , j+1, k+1)]; // north
    const std::string label = "cc_r" + std::to_string(r) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k);
    IloRange constraint(env, -IloInfinity, c_expr, 0.0, label.c_str());
    con.add(constraint);
    c_expr.end();
}

/**
 * @brief FinderPoints::constraint_volume
 * Ensure that robots r1 and r2 are not both at point (i,j) at time k
 */
void FinderPoints::constraint_coincidence(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const
{
    if (r1 == r2)
        return;

    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    int count = 0;
    if (is_var(r1,i,j,k)) { c_expr += x[index(r1,i,j,k)]; count++; }
    if (is_var(r2,i,j,k)) { c_expr += x[index(r2,i,j,k)]; count++; }
    if (count == 2)
    {
        const std::string label = "cv_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k);
        IloRange constraint(env, -IloInfinity, c_expr, 1.0, label.c_str());
        con.add(constraint);
    //    std::cout << constraint << std::endl;
    }
    c_expr.end();
}


void FinderPoints::constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const
{
    if (!(k+1 <= m_k))
        return;
    if (r1 == r2)
        return;

    IloEnv env = model.getEnv();
    int count;
    /* east */
    IloExpr c_expr_e(env);
    count = 0;
    if (is_var(r1,i  ,j,k  )) { c_expr_e += x[index(r1,i  ,j,k  )]; count++; }
    if (is_var(r2,i-1,j,k  )) { c_expr_e += x[index(r2,i-1,j,k  )]; count++; }
    if (is_var(r1,i-1,j,k+1)) { c_expr_e += x[index(r1,i-1,j,k+1)]; count++; }
    if (is_var(r2,i-2,j,k+1)) { c_expr_e -= x[index(r2,i-2,j,k+1)]; }
    if (count == 3)
    {
        const std::string label_e = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k) + "_east";
        IloRange constraint_e(env, -IloInfinity, c_expr_e, 2.0, label_e.c_str());
        con.add(constraint_e);
    }
    c_expr_e.end();
    /* west */
    IloExpr c_expr_w(env);
    count = 0;
    if (is_var(r1,i  ,j,k  )) { c_expr_w += x[index(r1,i  ,j,k  )]; count++; }
    if (is_var(r2,i+1,j,k  )) { c_expr_w += x[index(r2,i+1,j,k  )]; count++; }
    if (is_var(r1,i+1,j,k+1)) { c_expr_w += x[index(r1,i+1,j,k+1)]; count++; }
    if (is_var(r2,i+2,j,k+1)) { c_expr_w -= x[index(r2,i+2,j,k+1)]; }
    if (count == 3)
    {
        const std::string label_w = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k) + "_west";
        IloRange constraint_w(env, -IloInfinity, c_expr_w, 2.0, label_w.c_str());
        con.add(constraint_w);
    }
    c_expr_w.end();
    /* south */
    IloExpr c_expr_s(env);
    count = 0;
    if (is_var(r1,i,j  ,k  )) { c_expr_s += x[index(r1,i,j  ,k  )]; count++; }
    if (is_var(r2,i,j-1,k  )) { c_expr_s += x[index(r2,i,j-1,k  )]; count++; }
    if (is_var(r1,i,j-1,k+1)) { c_expr_s += x[index(r1,i,j-1,k+1)]; count++; }
    if (is_var(r2,i,j-2,k+1)) { c_expr_s -= x[index(r2,i,j-2,k+1)]; }
    if (count == 3)
    {
        const std::string label_s = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k) + "_south";
        IloRange constraint_s(env, -IloInfinity, c_expr_s, 2.0, label_s.c_str());
        con.add(constraint_s);
    }
    c_expr_s.end();
    /* north */
    IloExpr c_expr_n(env);
    count = 0;
    if (is_var(r1,i,j  ,k  )) { c_expr_n += x[index(r1,i,j  ,k  )]; count++; }
    if (is_var(r2,i,j+1,k  )) { c_expr_n += x[index(r2,i,j+1,k  )]; count++; }
    if (is_var(r1,i,j+1,k+1)) { c_expr_n += x[index(r1,i,j+1,k+1)]; count++; }
    if (is_var(r2,i,j+2,k+1)) { c_expr_n -= x[index(r2,i,j+2,k+1)]; }
    if (count == 3)
    {
        const std::string label_n = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k) + "_north";
        IloRange constraint_n(env, -IloInfinity, c_expr_n, 2.0, label_n.c_str());
        con.add(constraint_n);
    }
    c_expr_n.end();
}


int FinderPoints::index(int r, int i, int j, int k) const
{
    const Tuple tup(r, i, j, k);
    assert(m_index.count(tup) > 0);
    return m_index.at(tup);
}


void FinderPoints::make_solution(IloNumArray vals)
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point &s = m_ins.s(r);
        for (int k = 0; k <= m_k; k++)
            for (int i = s.x-k; i <= s.x+k; i++)
                for (int j = s.y-k; j <= s.y+k; j++)
                    if (is_var(r,i,j,k) && vals[index(r,i,j,k)] > .5)
                        m_sol.add_robot_position(r, Point(i, j));
    }
}


/**
 * @brief FinderPoints::solve
 * Make the optimization problem and solve it.
 * You should not optimize makespan and distance at the same time
 */
bool FinderPoints::solve_sat(int makespan)
{
    if (makespan < m_ins.makespan_bound())
        return false;

    /* reset */
    m_sol.clear_steps();
    set_k(makespan);
    m_index.clear();
    m_tuple.clear();

    make_variables();
    make_clauses();
    write_dimacs();

    /// @todo
    return true;
}


void FinderPoints::make_variables()
{
    // add the boolean variables x_{r,i,j,k}
    int index = 0;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_ins.s(r);
        const Point t = m_ins.t(r);
        for (int k = 0; k <= m_k; k++)
        {
            for (int i = s.x-k; i <= s.x+k; i++)
            {
                for (int j = s.y-k; j <= s.y+k; j++)
                {
                    const Point p(i, j);
                    if (p.dist(s) <= k && p.dist(t) <= m_k-k && !m_ins.is_obstacle(p))
                    {
                        const Tuple tup(r, i, j, k);
                        assert(m_index.count(tup) == 0);
                        m_index[tup] = index;
                        m_tuple.push_back(tup);
                        index++;
                    }
                }
            }
        }
    }
}


void FinderPoints::make_clauses()
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_ins.s(r);
        clause_start_target(r);
        for (int k = 0; k <= m_k; k++)
        {
            clause_uniqueness(r, k);
            for (int i = s.x-k; i <= s.x+k; i++)
            {
                for (int j = s.y-k; j <= s.y+k; j++)
                {
                    clause_continuity(r, i, j, k);
                    for (int r2 = 0; r2 < m_ins.nb_robots(); r2++)
                    {
                        clause_coincidence(r, r2, i, j, k);
                        clause_overlap(r, r2, i, j, k);
                    }
                }
            }
        }
    }
}


void FinderPoints::clause_uniqueness(int r, int k)
{

    const Point &s = m_ins.s(r);
    std::vector<std::pair<int, int>> pairs;
    for (int i = s.x-k; i <= s.x+k; ++i)
        for (int j = s.y-k; j <= s.y+k; ++j)
            if (is_var(r, i, j ,k))
                pairs.push_back(std::pair<int, int>(i, j));
    for (const auto &pair1 : pairs)
        for (const auto &pair2 : pairs)
            if (index(r, pair1.first, pair1.second, k) < index(r, pair2.first, pair2.second, k))
            {
                std::list<int> clause;
                clause.push_back(-index(r, pair1.first, pair1.second, k)-1);
                clause.push_back(-index(r, pair2.first, pair2.second, k)-1);
                m_cnf.push_back(clause);
            }
}

void FinderPoints::clause_start_target(int r)
{
    const Point start = m_ins.s(r);
    std::list<int> clause_start;
    clause_start.push_back(index(r, start.x, start.y, 0)+1);
    m_cnf.push_back(clause_start);

    const Point target = m_ins.t(r);
    std::list<int> clause_target;
    clause_target.push_back(index(r, target.x, target.y, m_k)+1);
    m_cnf.push_back(clause_target);
}


void FinderPoints::clause_continuity(int r, int i, int j, int k)
{
    if (!(k+1 <= m_k))
        return;

    std::list<int> clause;
    if (is_var(r,i  ,j  ,k  )) clause.push_back(-index(r, i  , j  , k  )-1); // initial position
    else return; // if the current position is not a variable, this clause must not be set
    if (is_var(r,i  ,j  ,k+1)) clause.push_back( index(r, i  , j  , k+1)+1); // same at next step
    if (is_var(r,i-1,j  ,k+1)) clause.push_back( index(r, i-1, j  , k+1)+1); // west
    if (is_var(r,i+1,j  ,k+1)) clause.push_back( index(r, i+1, j  , k+1)+1); // east
    if (is_var(r,i  ,j-1,k+1)) clause.push_back( index(r, i  , j-1, k+1)+1); // south
    if (is_var(r,i  ,j+1,k+1)) clause.push_back( index(r, i  , j+1, k+1)+1); // north
    if (!clause.empty())
        m_cnf.push_back(clause);
}


void FinderPoints::clause_coincidence(int r1, int r2, int i, int j, int k)
{
    if (r1 == r2)
        return;

    std::list<int> clause;
    if (is_var(r1,i,j,k)) clause.push_back(-index(r1,i,j,k)-1);
    if (is_var(r2,i,j,k)) clause.push_back(-index(r2,i,j,k)-1);
    if (clause.size() == 2)
        m_cnf.push_back(clause);
}

void FinderPoints::clause_overlap(int r1, int r2, int i, int j, int k)
{
    if (!(k+1 <= m_k))
        return;
    if (r1 == r2)
        return;

    /* east */
    std::list<int> clause;
    if (is_var(r1,i  ,j,k  )) clause.push_back(-index(r1,i  ,j,k  )-1);
    if (is_var(r2,i-1,j,k  )) clause.push_back(-index(r2,i-1,j,k  )-1);
    if (is_var(r1,i-1,j,k+1)) clause.push_back(-index(r1,i-1,j,k+1)-1);
    if (clause.size() == 3)
    {
        if (is_var(r2,i-2,j,k+1)) clause.push_back( index(r2,i-2,j,k+1)+1);
        m_cnf.push_back(clause);
    }
    /* west */
    clause.clear();
    if (is_var(r1,i  ,j,k  )) clause.push_back(-index(r1,i  ,j,k  )-1);
    if (is_var(r2,i+1,j,k  )) clause.push_back(-index(r2,i+1,j,k  )-1);
    if (is_var(r1,i+1,j,k+1)) clause.push_back(-index(r1,i+1,j,k+1)-1);
    if (clause.size() == 3)
    {
        if (is_var(r2,i+2,j,k+1)) clause.push_back( index(r2,i+2,j,k+1)+1);
        m_cnf.push_back(clause);
    }
    /* south */
    clause.clear();
    if (is_var(r1,i,j  ,k  )) clause.push_back(-index(r1,i,j  ,k  )-1);
    if (is_var(r2,i,j-1,k  )) clause.push_back(-index(r2,i,j-1,k  )-1);
    if (is_var(r1,i,j-1,k+1)) clause.push_back(-index(r1,i,j-1,k+1)-1);
    if (clause.size() == 3)
    {
        if (is_var(r2,i,j-2,k+1)) clause.push_back( index(r2,i,j-2,k+1)+1);
        m_cnf.push_back(clause);
    }
    /* north */
    clause.clear();
    if (is_var(r1,i,j  ,k  )) clause.push_back(-index(r1,i,j  ,k  )-1);
    if (is_var(r2,i,j+1,k  )) clause.push_back(-index(r2,i,j+1,k  )-1);
    if (is_var(r1,i,j+1,k+1)) clause.push_back(-index(r1,i,j+1,k+1)-1);
    if (clause.size() == 3)
    {
        if (is_var(r2,i,j+2,k+1)) clause.push_back( index(r2,i,j+2,k+1)+1);
        m_cnf.push_back(clause);
    }
}


void FinderPoints::write_dimacs() const
{
    std::ofstream file("clauses.cnf", std::ifstream::out | std::ifstream::binary);

    file << "c DIMACS file generated by Aldo GL" << std::endl;
    file << "p cnf " << m_tuple.size() << " " << m_cnf.size() << std::endl;
    for (const std::list<int> &clause : m_cnf)
    {
        for (const int x : clause)
        {
            file << x << " ";
        }
        file << "0" << std::endl;
    }
}
