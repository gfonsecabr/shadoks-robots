#include "searchermoves.h"

#include <queue>

SearcherMoves::SearcherMoves(const Instance &ins, const Options &options, int steps) :
    m_ins(ins), m_opt(options), m_sol(ins), m_k(steps)
{
    /* initialize start position and assign to instance starts by default */
    m_start.resize(m_ins.nb_robots());
    for (int r = 0; r < m_ins.nb_robots(); r++)
        m_start.at(r) = m_ins.s(r);

    /* assign indices to end points */
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




void SearcherMoves::set_robot_position(int r, Point p)
{
    assert(0 <= r && r < m_ins.nb_robots());
    m_start.at(r) = p;
}

/**
 * @brief SearcherMoves::set_weight
 * @param r Index of the robot
 * @param w The weight of that endpoint on that robot
 */
void SearcherMoves::set_weight(int r, Point p, double w)
{
    const int i = m_index_of_point.at(p);
    set_weight(r, i, w);
}


void SearcherMoves::set_weight(int r, int i, double w)
{
    m_weights.at(r).at(i) = w;
}




/**
 * @brief SearcherMoves::solve
 * There exists a solution. If it does not find it, it exits the problem.
 */
void SearcherMoves::solve()
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

void SearcherMoves::add_variables(IloModel model, IloNumVarArray x) const
{
    IloEnv env = model.getEnv();
    // variables x_{r,k,m}
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int k = 0; k < m_k; k++)
        {
            for (int m = 0; m < 5; m++)
            {
                const std::string label = "x_r" + std::to_string(r) + "_k" + std::to_string(k) + "_m" + std::to_string(m);
                x.add(IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, label.c_str()));
            }
        }
    }
    // variables y_{r,i,j}
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (const Point &p : m_point_with_index)
        {
            const std::string label = "y_r" + std::to_string(r) + "_i" + std::to_string(p.x) + "_j" + std::to_string(p.y);
            x.add(IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, label.c_str()));
        }
    }
}


void SearcherMoves::add_objective_function(IloModel model, IloNumVarArray x) const
{
    IloEnv env = model.getEnv();
    IloExpr obj_expr(env);
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        int i = 0;
        for (const Point &p : m_point_with_index)
        {
            obj_expr += m_weights.at(r).at(i) * x[index_of_yrij(r, p)];
            i++;
        }
    }
    IloObjective obj = IloMaximize(env, obj_expr);
//    std::cout << obj << std::endl; // debugging
    obj_expr.end();
    model.add(obj);
}


void SearcherMoves::add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const
{
    for (int r = 0; r < m_ins.nb_robots(); ++r)
    {
        constraint_endpoints(model, x, con, r);
        constraint_obstacle(model, x, con, r);
    }

    for (int r1 = 0; r1 < m_ins.nb_robots(); ++r1)
        for (int k = 0; k < m_k; k++)
            constraint_existence_uniqueness(model, x, con, r1, k);

    for (int r1 = 0; r1 < m_ins.nb_robots(); ++r1)
    {
        for (int r2 = r1 + 1; r2 < m_ins.nb_robots(); ++r2)
        {
            constraint_volume(model, x, con, r1, r2);
            constraint_overlap(model, x, con, r1, r2);
        }
    }
//    std::cout << con << std::endl; // debugging
    model.add(con);
}


/**
 * @brief SearcherMoves::constraint_endpoints
 * Make all constraints so that each sequence of moves is identified to an endpoint
 *
 * @todo I think this is wrong and that paths are not indexed to integers in that way
 */
void SearcherMoves::constraint_endpoints(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const
{
    IloEnv env = model.getEnv();
    const int nb_paths = round(std::pow(5, m_k));
    for (int i = 0; i < nb_paths; i++) // for each possible sequence of moves
    {
        IloExpr c_expr(env);
        int index = i;
        Point p(0, 0);
        for (int k = 0; k < m_k; k++)
        {
            const int move = index % 5;
            c_expr += x[index_of_xrkm(r,k,move)];
            switch (move) {
            case 1: p.x--; break; // (-1, 0)
            case 2: p.x++; break; // (+1, 0)
            case 3: p.y--; break; // (0, -1)
            case 4: p.y++; break; // (0, +1)
            }
            index /= 5;
        }
        c_expr -= x[index_of_yrij(r, p)];
        const std::string label = "ce_r" + std::to_string(r) + "_i" + std::to_string(p.x) + "_j" + std::to_string(p.y);
        IloRange constraint(env, -IloInfinity, c_expr, m_k-1, label.c_str());
        con.add(constraint);
        c_expr.end();
    }
    /* Also, we ask the endpoints to be unique for each robot.
     * Otherwise, they can all be set to true and maximize the objective function
     */
    IloExpr c_expr(env);
    for (const Point &p : m_point_with_index)
        c_expr += x[index_of_yrij(r, p)];
    const std::string label = "ce_r" + std::to_string(r);
    IloRange constraint(env, -IloInfinity, c_expr, 1, label.c_str());
    con.add(constraint);
    c_expr.end();
}


/**
 * A robot must choose a move and there is only one.
 * It is important to choose a move, because that will allow us to tell the endpoint
 *
 * @todo However, I do not need to be unique because there are other constraints
 * that make the endpoint unique. This would remove some constraints, but I would
 * be careful when extracting the solution.
 */
void SearcherMoves::constraint_existence_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int k) const
{
    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    for (int m = 0; m < 5; m++)
    {
        c_expr += x[index_of_xrkm(r, k, m)];
    }
    const std::string label = "ceu_r" + std::to_string(r) + "_k" + std::to_string(k);
    IloRange constraint(env, 1.0, c_expr, 1.0, label.c_str());
//    IloRange constraint(env, 1.0, c_expr, IloInfinity, label.c_str()); // Non-unique. Is it helpful?
    con.add(constraint);
    c_expr.end();
}


void SearcherMoves::constraint_volume(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const
{
    /* if they are too distant, stop here */
    if (m_start.at(r1).dist(m_start.at(r2)) > 2*m_k)
        return;

    IloEnv env = model.getEnv();
    std::queue<std::list<int>> queue;
    queue.push(std::list<int>()); // add empty list of moves
    while (!queue.empty())
    {
        std::list<int> cur_moves = queue.front(); queue.pop();
        // get the points at the end of these two paths
        Point p1 = m_start.at(r1);
        Point p2 = m_start.at(r2);
        for (int mm : cur_moves)
        {
            p1.move(mm / 5);    // m/5 and m%5 are the two moves encoded in the integer
            p2.move(mm % 5);
        }
        if (p1 == p2)
        {
            /* add constraint for this collision */
            IloExpr c_expr(env);
            int k = 0;
            for (int mm : cur_moves)
            {
                c_expr += x[index_of_xrkm(r1, k, mm / 5)];
                c_expr += x[index_of_xrkm(r2, k, mm % 5)];
                k++;
            }
            const std::string label = "cv_r1" + std::to_string(r1) + "_r2" + std::to_string(r2);
            IloRange constraint(env, -IloInfinity, c_expr, 2*cur_moves.size()-1, label.c_str());
            con.add(constraint);
            c_expr.end();
        }
        else if ((int)cur_moves.size() + 1 <= m_k)
        {
            /* continue */
            for (int m = 0; m < 25; m++)
            {
                std::list<int> next_moves(cur_moves);
                next_moves.push_back(m);
                queue.push(next_moves);
            }
        }
    }
}


void SearcherMoves::constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const
{
    /* if they are too distant, stop here */
    if (m_start.at(r1).dist(m_start.at(r2)) >= 2*m_k)
        return;

    IloEnv env = model.getEnv();
    std::queue<std::list<int>> queue;
    queue.push(std::list<int>()); // add empty list of moves
    while (!queue.empty())
    {
        std::list<int> cur_moves = queue.front(); queue.pop();
        // get the points at the end of these two paths
        Point p1 = m_start.at(r1), p1_prev = p1;
        Point p2 = m_start.at(r2), p2_prev = p2;
        for (int mm : cur_moves)
        {
            p1_prev = p1; p1.move(mm / 5);    // m/5 and m%5 are the two moves encoded in the integer
            p2_prev = p2; p2.move(mm % 5);
        }
        if ( (p1 == p2_prev || p1_prev == p2) && !(p1-p2 == p1_prev-p2_prev))
        {
            /* add constraint for this overlap */
            IloExpr c_expr(env);
            int k = 0;
            for (int mm : cur_moves)
            {
                c_expr += x[index_of_xrkm(r1, k, mm / 5)];
                c_expr += x[index_of_xrkm(r2, k, mm % 5)];
                k++;
            }
            const std::string label = "co_r1" + std::to_string(r1) + "_r2" + std::to_string(r2);
            IloRange constraint(env, -IloInfinity, c_expr, 2*cur_moves.size()-1, label.c_str());
            con.add(constraint);
            c_expr.end();
        }
        else if ((int)cur_moves.size() + 1 <= m_k)
        {
            /* continue */
            for (int m = 0; m < 25; m++)
            {
                std::list<int> next_moves(cur_moves);
                next_moves.push_back(m);
                queue.push(next_moves);
            }
        }
    }
}


void SearcherMoves::constraint_obstacle(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const
{
    IloEnv env = model.getEnv();
    std::queue<std::list<int>> queue;
    queue.push(std::list<int>()); // add empty list of moves
    while (!queue.empty())
    {
        std::list<int> cur_moves = queue.front(); queue.pop();
        // get the point at the end of this path
        Point p = m_start.at(r);
        for (int m : cur_moves)
        {
            p.move(m);
        }
        if (m_ins.is_obstacle(p))
        {
            /* add constraint for this obstacle */
            IloExpr c_expr(env);
            int k = 0;
            for (int m : cur_moves)
            {
                c_expr += x[index_of_xrkm(r, k, m)];
                k++;
            }
            const std::string label = "cob_r" + std::to_string(r);
            IloRange constraint(env, -IloInfinity, c_expr, cur_moves.size()-1, label.c_str());
            con.add(constraint);
            c_expr.end();
        }
        else if ((int)cur_moves.size() + 1 <= m_k)
        {
            /* continue */
            for (int m = 0; m < 5; m++)
            {
                std::list<int> next_moves(cur_moves);
                next_moves.push_back(m);
                queue.push(next_moves);
            }
        }
    }
}

void SearcherMoves::initial_solution(IloCplex& cplex, const IloNumVarArray& x) const
{
    IloNumVarArray startVar(cplex.getEnv());
    IloNumArray startVal(cplex.getEnv());
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int k = 0; k < m_k; k++)
        {
            for (int m = 0; m < 5; m++)
            {
                startVar.add(x[index_of_xrkm(r,k,m)]);
                if (m == 0)
                    startVal.add(1.0);
                else
                    startVal.add(0.0);
            }
        }
    }
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (const Point &p : m_point_with_index)
        {
            startVar.add(x[index_of_yrij(r, p)]);
            if (p == Point(0, 0))
                startVal.add(1.0);
            else
                startVal.add(0.0);
        }
    }
    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
}


int SearcherMoves::index_of_xrkm(int r, int k, int m) const
{
    assert(0 <= r && r < m_ins.nb_robots());
    assert(0 <= k && k < m_k);
    assert(0 <= m && m < 5);
    int index =        m;
    index +=       k * 5;
    index += r * m_k * 5;
    return index;
}


int SearcherMoves::index_of_yrij(int r, Point p) const
{
    assert(0 <= r && r < m_ins.nb_robots());
    assert(p.dist(Point(0, 0)) <= m_k);
    int index = m_ins.nb_robots()*m_k*5;
    index += m_index_of_point.at(p);
    index += r * m_index_of_point.size();
    return index;
}


/**
 * @brief SearcherMoves::make_solution
 * Fill the solution
 */
void SearcherMoves::make_solution(IloNumArray vals)
{
    for (int r = 0; r < m_ins.nb_robots(); ++r)
        m_sol.add_robot_position(r, m_start.at(r));

    for (int k = 0; k < m_k; k++)
    {
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            for (int m = 0; m < 5; m++)
            {
                if (vals[index_of_xrkm(r,k,m)] > .5)
                {
                    Point p = m_sol.point(r, k);
                    p.move(m);
                    m_sol.add_robot_position(r, p);
                }
            }
        }
    }
}
