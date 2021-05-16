#include "searcherpoints.h"

SearcherPoints::SearcherPoints(const Instance &ins, const Options &options, int steps) :
    m_ins(ins), m_opt(options), m_sol(ins), m_k(steps)
{
    /* initialize start position and assign to instance starts by default */
    m_start.resize(m_ins.nb_robots());
    for (int r = 0; r < m_ins.nb_robots(); r++)
        m_start.at(r) = m_ins.s(r);

    /* assign indices to endpoints */
    for (int i = -m_k; i <= m_k; i++)
    {
        for (int j = -m_k; j <= m_k; j++)
        {
            const Point p(i,j);
            if (p.dist(Point(0,0)) <= m_k)
            {
                m_index_of_point[p] = m_point_with_index.size();
                m_point_with_index.push_back(p);
            }
        }
    }
    /* initialize weights */
    m_weights.resize(m_ins.nb_robots());
    for (int r = 0; r < m_ins.nb_robots(); r++)
        m_weights.at(r) = std::vector<double>(nb_endpoints(), 0.0);
}

void SearcherPoints::set_robot_position(int r, Point p)
{
    assert(0 <= r && r < m_ins.nb_robots());
    m_start.at(r) = p;
}

/**
 * @brief SearcherPoints::set_weight
 * @param r Index of the robot
 * @param w The weight of that endpoint on that robot
 */
void SearcherPoints::set_weight(int r, Point p, double w)
{
    const int i = m_index_of_point.at(p);
    set_weight(r, i, w);
}


void SearcherPoints::set_weight(int r, int i, double w)
{
    m_weights.at(r).at(i) = w;
}


/**
 * @brief SearcherPoints::solve
 * There exists a solution. If it does not find it, it exits the problem.
 */
void SearcherPoints::solve()
{
    IloEnv env;
    try {
        /* Build the model */
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);
        add_variables(model, var);
        add_objective_function(model, var);
        add_constraints(model, var, con);
        std::cout << m_ins.nb_robots() << " robots with paths of length " << m_k << std::endl;
        std::cout << var.getSize() << " variables, " << con.getSize() << " constraints." << std::endl;

        /* Solve the model */
        IloCplex cplex(model);
        initial_solution(cplex, var);
        if (m_opt.time_limit >= 0)
            cplex.setParam(IloCplex::Param::TimeLimit, m_opt.time_limit); // Bound the time spent on solving the problem
//        cplex.setParam(IloCplex::Param::Parallel, 1); // Deterministic approach, always the same solution
//        cplex.setOut(env.getNullStream());  // don't output stuff
//        cplex.setParam(IloCplex::Param::MIP::Display, 1); // Print few things
//        cplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0); // I am not sure that this is used
//        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap   , 0);
//        std::cout << model << std::endl; // debugging

        // Optimize the problem and obtain solution.
        if ( !cplex.solve() ) {
            std::cout << cplex.getStatus() << std::endl;
            env.error() << "Failed to optimize LP" << std::endl;
//            infeasibility(cplex, var, con); // show the conflicts that produce an infeasability
            assert(false);
            exit(EXIT_FAILURE);
        }

        /* Get the solution */
        IloNumArray vals(env);
        cplex.getValues(vals, var);
//        std::clog << "objective value: " << cplex.getObjValue() << std::endl;
//        std::clog << "values: " << vals << std::endl;
        make_solution(vals);
        m_sol.reduce_makespan();
        assert(m_sol.makespan() > 0);
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
}

/**
 * @brief SearcherPoints::add_variables
 * Add the variables to the model.
 * Also, make the indices for them.
 */
void SearcherPoints::add_variables(IloModel model, IloNumVarArray x)
{
    IloEnv env = model.getEnv();
    // variables x_{r,i,j,k}
    int index = 0;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_start.at(r);
        for (int k = 0; k <= m_k; k++)
        {
            for (int i = s.x-k; i <= s.x+k; i++)
            {
                for (int j = s.y-k; j <= s.y+k; j++)
                {
                    const Point p(i, j);
                    if (p.dist(s) <= k && !m_ins.is_obstacle(p))
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
 * @brief SearcherPoints::add_objective_function
 * The objective function is to maximize the sum of the weights on the endpoints
 */
void SearcherPoints::add_objective_function(IloModel model, IloNumVarArray x) const
{
    IloEnv env = model.getEnv();
    IloExpr obj_expr(env);
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_start.at(r);
        int i = 0;
        for (const Point &p : m_point_with_index)
        {
            if (is_var(r, s.x+p.x, s.y+p.y, m_k))
                obj_expr += m_weights.at(r).at(i) * x[index_of_xrijk(r, s.x+p.x, s.y+p.y, m_k)];
            i++;
        }
    }
    IloObjective obj = IloMaximize(env, obj_expr);
//    std::cout << obj << std::endl; // debugging
    obj_expr.end();
    model.add(obj);
}


void SearcherPoints::add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_start.at(r);
        constraint_start(model, x, con, r);
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
//    std::cout << con << std::endl; // debugging
    model.add(con);
}


void SearcherPoints::constraint_start(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const
{
    IloEnv env = model.getEnv();
    const Point s = m_start.at(r);
    const std::string label = "cs_r" + std::to_string(r);
    IloRange constraint_s(env, 1.0, x[index_of_xrijk(r, s.x, s.y, 0)], 1.0, label.c_str());
    con.add(constraint_s);
}

/**
 * Add constraints that ensure that for each robot and each time step, it occupies a single position
 * @note I do not need the existence part (given by start + neighborhood) and I can split the uniqueness
 */
void SearcherPoints::constraint_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const
{
    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    const Point s = m_start.at(r);
    for (int i = s.x-k; i <= s.x+k; ++i)
    {
        for (int j = s.y-k; j <= s.y+k; ++j)
        {
            if (is_var(r,i,j,k))
                c_expr += x[index_of_xrijk(r,i,j,k)];
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
 * Is this faster?
 */
void SearcherPoints::constraint_uniqueness_splitted(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const
{
    IloEnv env = model.getEnv();
    const std::string label = "cu_r" + std::to_string(r) + "_k" + std::to_string(k);
    const Point s = m_start.at(r);
    std::vector<std::pair<int, int>> pairs;
    for (int i = s.x-k; i <= s.x+k; ++i)
        for (int j = s.y-k; j <= s.y+k; ++j)
            if (is_var(r, i, j ,k))
                pairs.push_back(std::pair<int, int>(i, j));
    for (const auto &pair1 : pairs)
        for (const auto &pair2 : pairs)
            if (index_of_xrijk(r, pair1.first, pair1.second, k) < index_of_xrijk(r, pair2.first, pair2.second, k))
            {
                IloExpr c_expr(env);
                c_expr += x[index_of_xrijk(r, pair1.first, pair1.second, k)];
                c_expr += x[index_of_xrijk(r, pair2.first, pair2.second, k)];
                IloRange constraint(env, -IloInfinity, c_expr, 1.0, label.c_str());
                con.add(constraint);
                c_expr.end();
            }
}

/**
 * @brief SearcherPoints::constraint_neighborhood
 * Add the contraint that ensures that at each time, each robot stays in its 4-neighborhood.
 * If a robot is in a given position, at the next step it must be in some of the 5 possible positions
 * The constraint is: x - y1 ... -y5 <= 0
 * Hence, if the robot is there, the same robot must also be in any of the 5 places
 */
void SearcherPoints::constraint_continuity(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i, int j, int k) const
{
    if (!(k+1 <= m_k))
        return;

    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    if (is_var(r, i, j, k)) // initial position
        c_expr += x[index_of_xrijk(r, i, j, k)];
    else
        return; // only add constraint if that variable exist
    if (is_var(r,i  ,j  ,k+1)) c_expr -= x[index_of_xrijk(r, i  , j  , k+1)]; // same at next step
    if (is_var(r,i-1,j  ,k+1)) c_expr -= x[index_of_xrijk(r, i-1, j  , k+1)]; // west
    if (is_var(r,i+1,j  ,k+1)) c_expr -= x[index_of_xrijk(r, i+1, j  , k+1)]; // east
    if (is_var(r,i  ,j-1,k+1)) c_expr -= x[index_of_xrijk(r, i  , j-1, k+1)]; // south
    if (is_var(r,i  ,j+1,k+1)) c_expr -= x[index_of_xrijk(r, i  , j+1, k+1)]; // north
    const std::string label = "cc_r" + std::to_string(r) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k);
    IloRange constraint(env, -IloInfinity, c_expr, 0.0, label.c_str());
    con.add(constraint);
    c_expr.end();
}

/**
 * @brief SearcherPoints::constraint_volume
 * Ensure that robots r1 and r2 are not both at point (i,j) at time k
 */
void SearcherPoints::constraint_coincidence(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const
{
    /* only if r1 < r2 */
    if (r1 >= r2)
        return;

    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    int count = 0;
    if (is_var(r1,i,j,k)) { c_expr += x[index_of_xrijk(r1,i,j,k)]; count++; }
    if (is_var(r2,i,j,k)) { c_expr += x[index_of_xrijk(r2,i,j,k)]; count++; }
    if (count == 2)
    {
        const std::string label = "cv_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k);
        IloRange constraint(env, -IloInfinity, c_expr, 1.0, label.c_str());
        con.add(constraint);
    //    std::cout << constraint << std::endl;
    }
    c_expr.end();
}


void SearcherPoints::constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2, int i, int j, int k) const
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
    if (is_var(r1,i  ,j,k  )) { c_expr_e += x[index_of_xrijk(r1,i  ,j,k  )]; count++; }
    if (is_var(r2,i-1,j,k  )) { c_expr_e += x[index_of_xrijk(r2,i-1,j,k  )]; count++; }
    if (is_var(r1,i-1,j,k+1)) { c_expr_e += x[index_of_xrijk(r1,i-1,j,k+1)]; count++; }
    if (is_var(r2,i-2,j,k+1)) { c_expr_e -= x[index_of_xrijk(r2,i-2,j,k+1)]; }
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
    if (is_var(r1,i  ,j,k  )) { c_expr_w += x[index_of_xrijk(r1,i  ,j,k  )]; count++; }
    if (is_var(r2,i+1,j,k  )) { c_expr_w += x[index_of_xrijk(r2,i+1,j,k  )]; count++; }
    if (is_var(r1,i+1,j,k+1)) { c_expr_w += x[index_of_xrijk(r1,i+1,j,k+1)]; count++; }
    if (is_var(r2,i+2,j,k+1)) { c_expr_w -= x[index_of_xrijk(r2,i+2,j,k+1)]; }
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
    if (is_var(r1,i,j  ,k  )) { c_expr_s += x[index_of_xrijk(r1,i,j  ,k  )]; count++; }
    if (is_var(r2,i,j-1,k  )) { c_expr_s += x[index_of_xrijk(r2,i,j-1,k  )]; count++; }
    if (is_var(r1,i,j-1,k+1)) { c_expr_s += x[index_of_xrijk(r1,i,j-1,k+1)]; count++; }
    if (is_var(r2,i,j-2,k+1)) { c_expr_s -= x[index_of_xrijk(r2,i,j-2,k+1)]; }
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
    if (is_var(r1,i,j  ,k  )) { c_expr_n += x[index_of_xrijk(r1,i,j  ,k  )]; count++; }
    if (is_var(r2,i,j+1,k  )) { c_expr_n += x[index_of_xrijk(r2,i,j+1,k  )]; count++; }
    if (is_var(r1,i,j+1,k+1)) { c_expr_n += x[index_of_xrijk(r1,i,j+1,k+1)]; count++; }
    if (is_var(r2,i,j+2,k+1)) { c_expr_n -= x[index_of_xrijk(r2,i,j+2,k+1)]; }
    if (count == 3)
    {
        const std::string label_n = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2) + "_i" + std::to_string(i) + "_j" + std::to_string(j) + "_k" + std::to_string(k) + "_north";
        IloRange constraint_n(env, -IloInfinity, c_expr_n, 2.0, label_n.c_str());
        con.add(constraint_n);
    }
    c_expr_n.end();
}

void SearcherPoints::initial_solution(IloCplex& cplex, const IloNumVarArray& x) const
{
    IloNumVarArray startVar(cplex.getEnv());
    IloNumArray startVal(cplex.getEnv());
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        const Point s = m_start.at(r);
        for (int k = 0; k < m_k; k++)
        {
            for (int i = s.x-k; i <= s.x+k; ++i)
            {
                for (int j = s.y-k; j <= s.y+k; ++j)
                {
                    if (is_var(r,i,j,k))
                    {
                        startVar.add(x[index_of_xrijk(r, i, j, k)]);
                        if (i == s.x && j == s.y)
                            startVal.add(1.0);
                        else
                            startVal.add(0.0);
                    }
                }
            }
        }
    }
    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
}


void SearcherPoints::make_solution(IloNumArray vals)
{
    for (int k = 0; k <= m_k; ++k)
    {
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            const Point s = m_start.at(r);
            for (int i = s.x-k; i <= s.x+k; ++i)
            {
                for (int j = s.y-k; j <= s.y+k; ++j)
                {
                    if (is_var(r,i,j,k) && vals[index_of_xrijk(r,i,j,k)] > .5)
                    {
                        m_sol.add_robot_position(r, Point(i, j));
                    }
                }
            }
        }
    }
}


int SearcherPoints::index_of_xrijk(int r, int i, int j, int k) const
{
    const Tuple tup(r, i, j, k);
    assert(m_index.count(tup) > 0);
    return m_index.at(tup);
}
