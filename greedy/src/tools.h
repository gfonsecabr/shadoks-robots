#ifndef TOOLS_H
#define TOOLS_H

#include <cmath>        // std::abs
#include <string>

struct Options
{
    std::string instance = "";
    std::string solution = "";
    int time_limit = -1;
    int steps_ahead = 3;
    int time = 0;
    int space = 0;
    bool reverse = false;
    int seed = 0;
    std::string comment() const {
        std::string str = "Greedy algorithm for makespan";
        str += " with time=" + std::to_string(time) + ", ";
        str += "space=" + std::to_string(space) + ", ";
        str += "time limit=" + std::to_string(time_limit) + " seconds. ";
        return str;
    }
};

struct Point
{
    int x, y;
    Point () : x(0), y(0) {}
    Point (int _x, int _y) : x(_x), y(_y) {}
    Point neighbor(int i) const
    {
        if (i == 0) return Point(x+1, y  );
        if (i == 1) return Point(x  , y+1);
        if (i == 2) return Point(x-1, y  );
        else        return Point(x  , y-1);
    }
    int dist(const Point& p) const         { return std::abs(x - p.x) + std::abs(y - p.y); }
    int dist_max(const Point& p) const     { return std::max(std::abs(p.x - x), std::abs(p.y - y)); }
    void move(int m)
    {
        if (m == 1) x--;
        else if (m == 2) x++;
        else if (m == 3) y--;
        else if (m == 4) y++;
    }
    Point add(const Point& p) const        { return Point(x + p.x, y + p.y); }
    Point operator+(const Point& p) const  { return Point(x + p.x, y + p.y); }
    Point operator-(const Point& p) const  { return Point(x - p.x, y - p.y); }
    std::string str() const                { return "(" + std::to_string(x) + "," + std::to_string(y) + ")"; }
    bool operator ==(const Point& p) const { return p.x == x && p.y == y; }
    bool operator !=(const Point& p) const { return !(p.x == x && p.y == y); }
    bool operator <(const Point& p) const  { return (x < p.x) || (p.x == x && y < p.y); }
};


struct Box
{
    Point p1;
    int k1;
    Point p2;
    int k2;
    Box() : p1(0, 0), k1(0), p2(0, 0), k2(0) {}
    Box(Point _p1, int _k1, Point _p2, int _k2) : p1(_p1), k1(_k1), p2(_p2), k2(_k2) {}
    Point size() const { return Point(p2.x - p1.x + 1, p2.y - p1.y + 1); }
    void include(Point p) {
        p1.x = std::min(p1.x, p.x);
        p1.y = std::min(p1.y, p.y);
        p2.x = std::max(p2.x, p.x);
        p2.y = std::max(p2.y, p.y);
    }
    void include(int k) {
        k1 = std::min(k1, k);
        k2 = std::max(k2, k);
    }
    void include(Point p, int k) { include(p); include(k); }
    bool inside(Point p) const { return (p1.x <= p.x && p.x <= p2.x && p1.y <= p.y && p.y <= p2.y); }
    bool inside(int k) const { return (k1 <= k && k <= k2); }
    bool inside(Point p, int k) const {return inside(p) && inside(k); }
};

#endif // TOOLS_H
