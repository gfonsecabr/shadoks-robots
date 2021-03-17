// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Class that stores a robot
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
#ifndef ROBOT
#define ROBOT

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <algorithm>
#include "primitives.cpp"
#include "navigator.cpp"

using namespace std;

class Robot {
  int id;
  Point start, target;
public:
  vector<Point> path;
  RotateNavigator *nav = NULL, *navrev = NULL;

  Robot(const Point _start, const Point _target, const int _id)
  : id(_id), start(_start), target(_target) {
  }

  Robot(const Robot &r)
  : id(r.id), start(r.start), target(r.target), path(r.path), nav(r.nav), navrev(r.navrev) {
  }

  void setStart(const Point _start) {
    start = _start;
  }

  void setTarget(const Point _target) {
    target = _target;
  }

  int getId() const {
    return id;
  }

  Point getTarget() const{
    return target;
  }

  void rotate(int rot = 1) {
    start.rotate(rot);
    target.rotate(rot);
    if(nav != NULL)
      nav->rotate(rot);
    if(navrev != NULL)
      navrev->rotate(rot);
    for(Point &p : path)
      p.rotate(rot);
  }

  void reverse() {
    swap(start, target);
    swap(nav, navrev);
    std::reverse(path.begin(),path.end());
  }

  int time() const {
    return path.size() - 1;
  }

  Point getStart() const{
    return start;
  }

  void reset() {
    path.clear();
  }

//   int todo(int t = -1) const {
//     return pos(t).l1(target);
//   }

  Point pos(int t = -1) const {
    if(!started())
      return start;

    if(t == -1 || t >= path.size())
      t = path.size() - 1;

    return path[t];
  }

  bool compatible(Robot *r) const{
    int maxt = max(r->path.size(), path.size());
    for(int t = 0; t <= maxt; t++) {
      Move m(pos(t), pos(t+1), t);
      Move mr(r->pos(t), r->pos(t+1), t);
      if(!m.compatible(mr)) {
        return false;
      }
    }

    return true;
  }

  bool compatible(const Move &mr) const {
    int t = mr.t0;
    Move m(pos(t), pos(t+1), t);
    if(!m.compatible(mr)) {
      return false;
    }

    return true;
  }

  int makespan() const {
    if(!started())
      return 0;

    for(int i = path.size() - 1; i >= 1; i--) {
      if(path[i] != path[i-1])
        return i;
    }

    return 0;
  }

  void uniformPath(int mks) {
    while(path.size() < mks + 1) {
      path.push_back(path.back());
    }

    while(path.size() > mks + 1) {
      path.pop_back();
    }

//     cout << *this << " size " << path.size() << " mks " << makespan() << endl;

  }

  int distance() const {
    int x = 0;

    for(int i = 0; i < path.size()-1; i++) {
      x += path[i] != path[i+1];
    }

    return x;
  }

  int makespanLB() const {
    if(nav == NULL)
      return start.l1(target);
    return nav->dist(start);
  }

  bool started() const {
    return !path.empty();
  }

  bool arrived() const {
    return started() && pos() == target;
  }

  void add(Point p) {
    path.push_back(p);
  }

  friend ostream& operator<<(ostream& os, const Robot& r) {
//     os << r.id << ":" << r.start << "->" << r.pos() << "->" << r.target;;
    os << r.id << ":" << r.start << "->" << r.target;;
    return os;
  }

  void showPath() {
    for(Point p : path)
      cout << p;
    cout << endl;
  }

};

class Stinky : public Valuer {
  vector<Robot *> stinkyRobots;

public:
  Stinky(vector<Robot *> v) : stinkyRobots(v) {}
  
  virtual double get(Point key, int t) {
    double ret = 1.0;
    for(Robot *r : stinkyRobots) {
      ret += 1.0 / (1 + r->pos(t).l1(key));
    }
    return ret;
  }
};


#endif


