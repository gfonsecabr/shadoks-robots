// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Elementary classes
//
/**
 * MIT License
 *
 * Copyright (c) 2020 Guilherme Dias da Fonseca <gfonsecabr@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef PRIMITIVES
#define PRIMITIVES

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <functional>
#include <string>
#include <iomanip>      // std::put_time
#include <boost/unordered_set.hpp> // hash_combine
#include "tsl/hopscotch_map.h"
#include "tsl/hopscotch_set.h"

typedef short int pint;

using namespace std;

class Point {
public:
  pint x,y;

  Point() {
  }

  Point(pint _x, pint _y) : x(_x), y(_y) {
  }

  int l1(const Point &p = Point(0,0)) const {
    return abs(x-p.x) + abs(y-p.y);
  }

  int linf(const Point &p = Point(0,0)) const {
    return max(abs(x-p.x), abs(y-p.y));
  }

  friend bool operator==(const Point &p, const Point &q) {
    return p.x == q.x && p.y == q.y;
  }

  auto operator<=>(const Point& p) const {
    return make_tuple(x,y) <=> make_tuple(p.x,p.y);
  }

  Point operator-(const Point &p) const {
    return Point(x-p.x, y-p.y);
  }

  Point operator+(const Point &p) const {
    return Point(x+p.x, y+p.y);
  }

  // Dot product
  int operator*(const Point &p) const {
    return x*p.x + y*p.y;
  }

  string toString() const {
    string s = "(" + to_string(x) + "," + to_string(y) + ")";
    return s;
  }

  void rotate(int rot = 1) {
    while(rot < 0)
      rot += 4;
    rot %= 4;
    for(int i = 0; i < rot; i++) {
      swap(x,y);
      x = -x;
    }
  }

  bool inside(Point minxy, Point maxxy) {
    return x >= minxy.x && x <= maxxy.x && y >= minxy.y && y <= maxxy.y;
  }

  friend ostream& operator<<(ostream& os, const Point& p) {
    os << p.toString();
    return os;
  }
};

namespace std {
  template <> struct hash<Point> {
    size_t operator()(const Point& p) const {
      size_t seed = 0;
      boost::hash_combine(seed, p.x);
      boost::hash_combine(seed, p.y);
      return seed;
    }
  };
}

class Move {
public:
  Point p,q;
  int t0;

  Move() {
  }

  Move(Point _p, Point _q, int _t0 = -1) : p(_p), q(_q), t0(_t0) {
  }

  bool compatible(const Move &m) {
    if(m.q == q || m.p == p) {
      return false;
    }
    if(m.p == q || m.q == p) {
      return m.q - m.p == q - p;
    }
    return true;
  }

  string toString() const {
     string s;
     s = p.toString() + "-[" + to_string(t0) + "]->" + q.toString();
     return s;
  }

  friend ostream& operator<<(ostream& os, const Move& m) {
    os << m.toString();
    return os;
  }

  vector<Move> conflicts() const {
    vector<Move> ret;
    const vector<Point> displacements{{Point(0,1), Point(0,-1), Point(1,0), Point(-1,0)}};

    // Append movements that end in q
    for(Point v : displacements) {
      Move m(*this);
      m.p = q + v;
      if(m.p != p) {
        ret.push_back(m);
      }
    }

    if(p.l1(q) != 0) {
      // add movements that end in p but comme from other directions
      for(Point v : displacements) {
        Move m(*this);
        m.p = p + v;
        m.q = p;
        if(p - q != m.p - m.q) {
          ret.push_back(m);
        }
      }
    }

    return ret;
  }

  bool operator==(const Move &m) const = default;

// needs -std=c++20
  auto operator<=>(const Move& m) const {
    return make_tuple(p.x,q.x,p.y,q.y,t0) <=> make_tuple(m.p.x, m.q.x, m.p.y, m.q.y, m.t0);
  }
};

namespace std {
  template <> struct hash<Move> {
    size_t operator()(const Move& m) const {
      size_t seed = 0;
      boost::hash_combine(seed, m.p.x);
      boost::hash_combine(seed, m.p.y);
      boost::hash_combine(seed, m.q.x);
      boost::hash_combine(seed, m.q.y);
      boost::hash_combine(seed, m.t0);
      return seed;
    }
  };
}

class Valuer {
public:
  virtual double get(Point key, int t) {return 1.0;}
};

Valuer theValuer;

class Randomner : public Valuer {
  tsl::hopscotch_map<Point, double> memory;

public:
  virtual double get(Point key, int t) {
    if(memory.contains(key))
      return memory[key];

    double r = ((double) rand() / (RAND_MAX)) + 1.0 ;
    memory[key] = r;
    return r;
  }
};

string timeString() {
  std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
  struct std::tm * ptm = std::localtime(&tt);
  std::ostringstream oss;
  oss << std::put_time(ptm, "%Y%m%d-%H%M%S");
  string s = oss.str();

  return s;
}

const auto start_time = std::chrono::steady_clock::now();

double elapsedSec() {
  auto cur_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = cur_time - start_time;
  return elapsed_seconds.count();
}

#endif
