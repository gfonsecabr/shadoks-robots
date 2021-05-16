#include "searcherboundedpaths.h"

SearcherBoundedPaths::SearcherBoundedPaths(const Instance &ins, const Solution &sol, const std::vector<bool> planned, int steps, const Options &options) :
    m_ins(ins), m_sol(sol), m_planned(planned), m_k(steps), m_opt(options)
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

    make_paths();
    precompute_coincidence();
    precompute_overlap();
}

/**
 * @brief SearcherBoundedPaths::make_paths
 * Initialize all the possible paths of length m_k
 */
void SearcherBoundedPaths::make_paths()
{
    m_nb_paths = round(std::pow(5, m_k));
    m_path_with_index.resize(m_nb_paths);
    for (int i = 0; i < m_nb_paths; i++)
    {
        std::vector<Point> moves(m_k);
        int index = i;
        for (int j = m_k-1; j >= 0; j--)
        {
            /* The order of the moves is very important. Leaving the null-move
             * at the end encourages CPLEX to make moves at the beginning in the
             * paths when the endpoint is close. This is practical because I will
             * only make the first move, so it is better if it goes somewhere.
             */
            switch (index % 5) {
            case 0: moves.at(j) = Point(-1, 0); break;
            case 1: moves.at(j) = Point( 1, 0); break;
            case 2: moves.at(j) = Point(0 ,-1); break;
            case 3: moves.at(j) = Point(0 , 1); break;
            case 4: moves.at(j) = Point(0 , 0); break;
            }
            index /= 5;
        }
        m_path_with_index.at(i) = moves;
        assert(index_of_path(std::list<Point>(moves.cbegin(), moves.cend())) == i);
    }
}

/**
 * @brief SearcherBoundedPaths::precompute_coincidence_incompatibilities
 * Precompute the pairs of paths that break the coincidence constraint for all
 * pairs of start points.
 */
void SearcherBoundedPaths::precompute_coincidence()
{
    const Point origin(0, 0);
    for (int i = -2*m_k; i <= 2*m_k; i++)
    {
        for (int j = -2*m_k; j <= 2*m_k; j++)
        {
            const Point p(i, j);
            if (origin != p && origin.dist(p) <= 2*m_k)
            {
                std::list<std::pair<int, int>> pairs;
                for (int i1 = 0; i1 < m_nb_paths; i1++)
                {
                    for (int i2 = 0; i2 < m_nb_paths; i2++)
                    {
                        const std::vector<Point> &moves1 = m_path_with_index.at(i1);
                        const std::vector<Point> &moves2 = m_path_with_index.at(i2);
                        Point p1 = origin;
                        Point p2 = p;
                        bool coincidence = false;
                        for (int k = 0; k < m_k && !coincidence; k++)
                        {
                            p1 = p1 + moves1.at(k);
                            p2 = p2 + moves2.at(k);
                            if (p1 == p2)
                                coincidence = true;
                        }
                        if (coincidence)
                        {
                            pairs.push_back(std::make_pair(i1, i2));
                        }
                    }
                }
                m_coincidence[p] = pairs;
            }
        }
    }
}


void SearcherBoundedPaths::precompute_overlap()
{
    const Point origin(0, 0);
    for (int i = -2*m_k; i <= 2*m_k; i++)
    {
        for (int j = -2*m_k; j <= 2*m_k; j++)
        {
            const Point p(i, j);
            if (origin != p && origin.dist(p) < 2*m_k)
            {
                std::list<std::pair<int, int>> pairs;
                for (int i1 = 0; i1 < m_nb_paths; i1++)
                {
                    for (int i2 = 0; i2 < m_nb_paths; i2++)
                    {
                        const std::vector<Point> &moves1 = m_path_with_index.at(i1);
                        const std::vector<Point> &moves2 = m_path_with_index.at(i2);
                        Point p1 = origin;
                        Point p2 = p;
                        bool overlap = false;
                        for (int k = 0; k < m_k && !overlap; k++)
                        {
                            const Point next1 = p1 + moves1.at(k);
                            const Point next2 = p2 + moves2.at(k);
                            if ((p1.dist(p2) == 1) && (next1 == p2))
                            {
                                overlap = !((p2.x - p1.x == next2.x - next1.x) && (p2.y - p1.y == next2.y - next1.y));
                            }
                            p1 = next1;
                            p2 = next2;
                        }
                        if (overlap)
                        {
                            pairs.push_back(std::make_pair(i1, i2));
                        }
                    }
                }
                m_overlap[p] = pairs;
            }
        }
    }
}


void SearcherBoundedPaths::set_robot_position(int r, Point p)
{
    assert(0 <= r && r < m_ins.nb_robots());
    m_start.at(r) = p;
}

/**
 * @brief SearcherBoundedPaths::set_planned
 * Set robot r as planned, so we cannot move it
 */
void SearcherBoundedPaths::set_planned(int r)
{
    m_planned.at(r) = true;
}


/**
 * @brief SearcherBoundedPaths::set_weight
 * @param r Index of the robot
 * @param p endpoint
 * @param w The weight of that path on that robot
 */
void SearcherBoundedPaths::set_weight(int r, Point p, double w)
{
    const int i = m_index_of_point.at(p);
    set_weight(r, i, w);
}


void SearcherBoundedPaths::set_weight(int r, int i, double w)
{
    m_weights.at(r).at(i) = w;
}

/**
 * @brief SearcherBoundedPaths::index_of_path
 * @param moves List of moves
 * @return The index of this path. For example, the index of [(0,1), (-1,0)] is 5*4 + 1
 */
int SearcherBoundedPaths::index_of_path(const std::list<Point> &moves) const
{
    assert((int)moves.size() == m_k);
    int index = 0;
    for (const Point &p : moves)
    {
        index *= 5;
        if (p == Point(1, 0)) index += 1;
        else if (p == Point(0, -1)) index += 2;
        else if (p == Point(0,  1)) index += 3;
        else if (p == Point(0,  0)) index += 4;
        else assert(p == Point(-1, 0));
    }
    return index;
}

/**
 * @brief SearcherBoundedPaths::index_of_path
 * @return The index of the path of a planned robot
 */
int SearcherBoundedPaths::index_of_path(int r) const
{
    assert(m_planned.at(r));
    std::list<Point> moves;
    for (int k = 0; k < m_k; k++)
    {
        const Point move = m_sol.point(r, k+1) - m_sol.point(r, k);
        moves.push_back(move);
    }
    return index_of_path(moves);
}


/**
 * @brief SearcherBoundedPaths::path_with_index
 * @return The sequence of moves associated to `index`
 * This function is used very often, so it is more efficient to compute the paths
 * only once and save them in memory.
 */
std::vector<Point> SearcherBoundedPaths::path_with_index(int index) const
{
    return m_path_with_index.at(index);
}

/**
 * @brief SearcherBoundedPaths::solve_naif
 * Similar to SearcherBoundedPaths::solve, but not the same
 * There exists a solution. If it does not find it, it exits the problem.
 */
bool SearcherBoundedPaths::solve()
{
    std::clog << "Solving with SearcherBoundedPaths in bbox ["<<m_bbox.p1.x<<","<<m_bbox.p2.x
              << "]x["<<m_bbox.p1.y<<","<<m_bbox.p2.y << "]" << std::endl;
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
//        cplex.setParam(IloCplex::Param::Parallel, 1); // Deterministic approach, always the same solution
        if (m_opt.time_limit > 0)
            cplex.setParam(IloCplex::Param::TimeLimit, m_opt.time_limit); // Bound the time spent on solving the problem
        cplex.setOut(env.getNullStream());  // don't output stuff
//        cplex.setParam(IloCplex::Param::MIP::Display, 1); // Print few things
        cplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0); // I am not sure that this is used
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap   , 0);
//        std::cout << model << std::endl; // debugging

        // Optimize the problem and obtain solution.
        if (!cplex.solve())
        {
            std::clog << cplex.getStatus() << std::endl;
            std::clog << "Failed to optimize LP" << std::endl;
            infeasibility(cplex, var, con); // show the conflicts that produce an infeasability
            return false;
        }

        /* Get the solution */
        IloNumArray vals(env);
        cplex.getValues(vals, var);
        make_solution(vals);
//        m_sol.reduce_makespan(); /// @note No, because this is a partial solution
        std::clog << "objective value (new current makespan): " << cplex.getObjValue() << std::endl;
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
 * @brief SearcherBoundedPaths::add_variables
 * Add variables x_{r,i} for each robot and path such that
 * - the robot is not planned
 * - if the path starts in the box, it remains in the box with the flaps
 * - it the path starts in a flap, it remains in a flap
 */
void SearcherBoundedPaths::add_variables(IloModel model, IloNumVarArray x)
{
    IloEnv env = model.getEnv();
    // robot-path variables x_{r,i}
    int index = 0;
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int i = 0; i < m_nb_paths; i++)
        {
            if (!m_planned.at(r) && (path_in_box_with_flaps(r, i) || path_in_flap(r, i)))
            {
                const Tuple tup(r, i);
                assert(m_index.count(tup) == 0);
                m_index[tup] = index;
                assert((int)m_tuple.size() == index);
                m_tuple.push_back(tup);
                index++;
                const std::string label = "x_r" + std::to_string(r) + "_i" + std::to_string(i);
                x.add(IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, label.c_str()));
            }
        }
    }
}


void SearcherBoundedPaths::add_objective_function(IloModel model, IloNumVarArray x) const
{
    IloEnv env = model.getEnv();
    IloExpr obj_expr(env);
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int i = 0; i < nb_paths(); i++)
        {
            if (is_var(r, i))
            {
                const double w = m_weights.at(r).at(m_index_of_point.at(endpoint(i)));
                obj_expr += w * x[index_of_xri(r, i)];
            }
        }
    }
    IloObjective obj = IloMaximize(env, obj_expr);
//    std::cout << obj << std::endl; // debugging
    obj_expr.end();
    model.add(obj);
}


void SearcherBoundedPaths::add_constraints(IloModel model, IloNumVarArray x, IloRangeArray con) const
{
    for (int r1 = 0; r1 < m_ins.nb_robots(); ++r1)
    {
        constraint_existence_uniqueness(model, x, con, r1);
        for (int r2 = 0; r2 < m_ins.nb_robots(); ++r2)
        {
            constraint_volume (model, x, con, r1, r2);
            constraint_overlap(model, x, con, r1, r2);
        }
        for (int i = 0; i < m_nb_paths; i++)
        {
            constraint_obstacle(model, x, con, r1, i);
        }
    }
    model.add(con);
}


/**
 * Add constraints that ensure that for each robot, only one path is chosen
 */
void SearcherBoundedPaths::constraint_existence_uniqueness(IloModel model, IloNumVarArray x, IloRangeArray con, int r) const
{
    IloEnv env = model.getEnv();
    IloExpr c_expr(env);
    bool empty = true;
    for (int i = 0; i < m_nb_paths; i++)
    {
        if (is_var(r, i))
        {
            c_expr += x[index_of_xri(r, i)];
            empty = false;
        }
    }
    if (!empty)
    {
        const std::string label = "ceu_r" + std::to_string(r);
        IloRange constraint(env, 1.0, c_expr, 1.0, label.c_str());
        con.add(constraint);
    }
    c_expr.end();
}

/**
 * @brief SearcherBoundedPaths::constraint_volume
 * Do not allow two paths on two robots that send them to the same position
 */
void SearcherBoundedPaths::constraint_volume(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const
{
    if (r1 == r2)
        return;

    /* if the robots are too distant, stop here */
    if (m_start.at(r1).dist(m_start.at(r2)) > 2*m_k)
        return;

    IloEnv env = model.getEnv();
    const Point p = m_start.at(r2) - m_start.at(r1);
    assert(m_coincidence.count(p) > 0);

    /* if the second robot is already planned */
    if (m_planned.at(r2))
    {
        const int i2 = index_of_path(r2);
        for (const std::pair<int, int> &pair : m_coincidence.at(p))
        {
            const int i1 = pair.first;
            if (i2 == pair.second && is_var(r1, i1))
            {
                const std::string label = "cv_r1" + std::to_string(r1)+ "_i1" + std::to_string(i1) + "_r2*" + std::to_string(r2)  + "_i2" + std::to_string(i2);
                IloRange constraint(env, -IloInfinity, x[index_of_xri(r1, i1)], 0.0, label.c_str());
                con.add(constraint);
            }
        }
    }
    else
    {
        for (const std::pair<int, int> &pair : m_coincidence.at(p))
        {
            const int i1 = pair.first;
            const int i2 = pair.second;
            if (is_var(r1, i1) && is_var(r2, i2))
            {
                const std::string label = "cv_r1" + std::to_string(r1)+ "_i1" + std::to_string(i1) + "_r2" + std::to_string(r2)  + "_i2" + std::to_string(i2);
                IloRange constraint(env, -IloInfinity, x[index_of_xri(r1, i1)] + x[index_of_xri(r2, i2)], 1.0, label.c_str());
                con.add(constraint);
            }
        }
    }
}

/**
 * If these choices of paths for both robots overlap, do not allow these two choices
 */
void SearcherBoundedPaths::constraint_overlap(IloModel model, IloNumVarArray x, IloRangeArray con, int r1, int r2) const
{
    if (r1 == r2)
        return;

    /* if they are too distant, stop here */
    if (m_start.at(r1).dist(m_start.at(r2)) >= 2*m_k)
        return;

    IloEnv env = model.getEnv();
    const Point p = m_start.at(r2) - m_start.at(r1);
    assert(m_overlap.count(p) > 0);

    /* if the second robot is already planned */
    if (m_planned.at(r2))
    {
        const int i2 = index_of_path(r2);
        for (const std::pair<int, int> &pair : m_overlap.at(p))
        {
            const int i1 = pair.first;
            if (i2 == pair.second && is_var(r1, i1))
            {
                const std::string label = "co_r1" + std::to_string(r1)+ "_i1" + std::to_string(i1) + "_r2*" + std::to_string(r2)  + "_i2" + std::to_string(i2);
                IloRange constraint(env, -IloInfinity, x[index_of_xri(r1, i1)], 0.0, label.c_str());
                con.add(constraint);
            }
        }
    }
    else if (m_planned.at(r1))
    {
        const int i1 = index_of_path(r1);
        for (const std::pair<int, int> &pair : m_overlap.at(p))
        {
            const int i2 = pair.second;
            if (i1 == pair.first && is_var(r2, i2))
            {
                const std::string label = "co_r1*" + std::to_string(r1)+ "_i1" + std::to_string(i1) + "_r2" + std::to_string(r2)  + "_i2" + std::to_string(i2);
                IloRange constraint(env, -IloInfinity, x[index_of_xri(r2, i2)], 0.0, label.c_str());
                con.add(constraint);
            }
        }
    }
    else
    {
        for (const std::pair<int, int> &pair : m_overlap.at(p))
        {
            const int i1 = pair.first;
            const int i2 = pair.second;
            if (is_var(r1, i1) && is_var(r2, i2))
            {
                const std::string label = "co_r1" + std::to_string(r1) + "_i1" + std::to_string(i1) + "_r2" + std::to_string(r2) + "_i2" + std::to_string(i2);
                IloRange constraint(env, -IloInfinity, x[index_of_xri(r1, i1)] + x[index_of_xri(r2, i2)], 1.0, label.c_str());
                con.add(constraint);
            }
        }
    }
}

void SearcherBoundedPaths::constraint_obstacle(IloModel model, IloNumVarArray x, IloRangeArray con, int r, int i) const
{
    if (!is_var(r, i))
        return;

    const std::vector<Point> &moves = m_path_with_index.at(i);
    Point p = m_start.at(r);
    bool obstacle = false;
    for (int j = 0; j < m_k && !obstacle; j++)
    {
        p = p + moves.at(j);
        obstacle = m_ins.is_obstacle(p);
    }
    if (obstacle)
    {
        IloEnv env = model.getEnv();
        const std::string label = "cob_r" + std::to_string(r) + "_i" + std::to_string(i);
        IloRange constraint(env, 0.0, x[index_of_xri(r, i)], 0.0, label.c_str());
        con.add(constraint);
    }
}







int SearcherBoundedPaths::index_of_xri(int r, int i) const
{
    const Tuple tup(r, i);
    assert(m_index.count(tup) > 0);
    return m_index.at(tup);
}

/**
 * Print the conflict: a minimal set of constraints that is infeasible
 * See https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/UsrMan/topics/infeas_unbd/conflict_refiner/23_appli_title_synopsis.html
 */
void SearcherBoundedPaths::infeasibility(IloCplex &cplex, IloNumVarArray &var, IloRangeArray &con) const
{
    if ( ( cplex.getStatus() == IloAlgorithm::Infeasible ) ||
         ( cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded ) ) {
        std::cout << std::endl << "No solution - starting Conflict refinement" << std::endl;
        IloEnv env = cplex.getEnv();
        IloConstraintArray infeas(env);
        IloNumArray preferences(env);
        infeas.add(con);
        for (IloInt i = 0; i<var.getSize(); i++) {
            if ( var[i].getType() != IloNumVar::Bool ) {
                infeas.add(IloBound(var[i], IloBound::Lower));
                infeas.add(IloBound(var[i], IloBound::Upper));
            }
        }
        for (IloInt i = 0; i<infeas.getSize(); i++) {
            preferences.add(1.0); // user may wish to assign unique preferences
        }
        cplex.setOut(env.getNullStream());  // don't output stuff
        if ( cplex.refineConflict(infeas, preferences) ) {
            IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
            env.getImpl()->useDetailedDisplay(IloTrue);
            std::cout << "Conflict :" << std::endl;
            for (IloInt i = 0; i<infeas.getSize(); i++) {
                if ( conflict[i] == IloCplex::ConflictMember)
                    std::cout << "Proved : " << infeas[i] << std::endl;
                if ( conflict[i] == IloCplex::ConflictPossibleMember)
                    std::cout << "Possible: " << infeas[i] << std::endl;
            }
        }
        else
            std::cout << "Conflict could not be refined" << std::endl;
        std::cout << std::endl;
    }
}


/**
 * @brief SearcherBoundedPaths::initial_solution
 * Provide a trivial valid solution, where robots do not move
 */
void SearcherBoundedPaths::initial_solution(IloCplex& cplex, const IloNumVarArray& x) const
{
    IloNumVarArray startVar(cplex.getEnv());
    IloNumArray startVal(cplex.getEnv());
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        for (int i = 0; i < nb_paths(); i++)
        {
            if (is_var(r, i))
            {
                startVar.add(x[index_of_xri(r, i)]);
                if (i == nb_paths()-1)
                    startVal.add(1.0);
                else
                    startVal.add(0.0);
            }
        }
    }
    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
}

/**
 * @brief SearcherBoundedPaths::make_solution
 * Fill the solution
 */
void SearcherBoundedPaths::make_solution(IloNumArray vals)
{
    for (int i = 0; i < m_nb_paths; i++)
    {
        const std::vector<Point> &moves = m_path_with_index.at(i);
        for (int r = 0; r < m_ins.nb_robots(); ++r)
        {
            if (is_var(r, i))
            {
                Point p = m_start.at(r);
                if (vals[index_of_xri(r,i)] > .5)
                {
                    m_sol.add_robot_position(r, p);
                    for (const Point &move : moves)
                    {
                        p = p + move;
                        m_sol.add_robot_position(r, p);
                    }
                }
            }
        }
    }
}

/**
 * @brief SearcherBoundedPaths::endpoint
 * @return The endpoint of the i-th path
 */
Point SearcherBoundedPaths::endpoint(int i) const
{
    Point e(0,0);
    const std::vector<Point> &path = m_path_with_index.at(i);
    for (const Point &p : path)
        e = e + p;
    return e;
}

/**
 * @brief SearcherBoundedPaths::point_in_box_with_flaps
 * @return True if the point is inside the bounding box or on its flaps
 */
bool SearcherBoundedPaths::point_in_box_with_flaps(Point p) const
{
    // the point is inside the bounding box
    if (m_bbox.inside(p))
        return true;
    // the point is in the horizontal flaps
    if (m_bbox.p1.x-m_flap <= p.x && p.x <= m_bbox.p2.x+m_flap && m_bbox.p1.y <= p.y && p.y <= m_bbox.p2.y)
        return true;
    // the point is in the vertical flaps
    if (m_bbox.p1.x <= p.x && p.x <= m_bbox.p2.x && m_bbox.p1.y-m_flap <= p.y && p.y <= m_bbox.p2.y+m_flap)
        return true;
    return false;
}

/**
 * @brief SearcherBoundedPaths::point_in_flap
 * @return True if the point is inside a flap
 */
bool SearcherBoundedPaths::point_in_flap(Point p) const
{
    // the point is not inside the box
    if (m_bbox.inside(p))
        return false;
    // the point is in the horizontal flaps
    if (m_bbox.p1.x-m_flap <= p.x && p.x <= m_bbox.p2.x+m_flap && m_bbox.p1.y <= p.y && p.y <= m_bbox.p2.y)
        return true;
    // the point is in the vertical flaps
    if (m_bbox.p1.x <= p.x && p.x <= m_bbox.p2.x && m_bbox.p1.y-m_flap <= p.y && p.y <= m_bbox.p2.y+m_flap)
        return true;
    return false;
}

/**
 * @brief SearcherBoundedPaths::path_in_box_with_flaps
 * @return True if the path (r,i) starts in the box and stays
 * inside the box with the flaps
 */
bool SearcherBoundedPaths::path_in_box_with_flaps(int r, int i) const
{
    const std::vector<Point> &moves = m_path_with_index.at(i);
    Point p = m_start.at(r);
    if (!m_bbox.inside(p))
        return false;
    for (const Point &move : moves)
    {
        p = p + move;
        if (!point_in_box_with_flaps(p))
            return false;
    }
    return true;
}

/**
 * @brief SearcherBoundedPaths::path_in_flap
 * @return True if the path (r,i) starts in a flap and stays inside a flap
 */
bool SearcherBoundedPaths::path_in_flap(int r, int i) const
{
    const std::vector<Point> &moves = m_path_with_index.at(i);
    Point p = m_start.at(r);
    if (!point_in_flap(p))
        return false;
    for (const Point &move : moves)
    {
        p = p + move;
        if (!point_in_flap(p))
            return false;
    }
    return true;
}


void SearcherBoundedPaths::print_weights() const
{
    for (int r = 0; r < m_ins.nb_robots(); r++)
    {
        std::cout << "r=" << r;
        for (std::size_t i = 0; i < m_weights.at(r).size(); ++i)
        {
            const Point &e = m_point_with_index.at(i);
            std::cout << " w" << e.str() << "=" << m_weights.at(r).at(i);
        }
        std::cout << std::endl;
    }
}


void SearcherBoundedPaths::clear_model()
{
    m_index.clear();
    m_tuple.clear();
}

