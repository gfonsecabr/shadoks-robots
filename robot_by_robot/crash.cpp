// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Class to detect collision of robots
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


#ifndef CRASH
#define CRASH

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <stack>
#include "robot.cpp"
#include <boost/unordered_set.hpp> // hash_combine
#include "tsl/hopscotch_map.h"
#include "tsl/hopscotch_set.h"

using namespace std;

// Hash for a pair of int and Point
struct int_point_hash : public unary_function<tuple<int,Point>, size_t> {
  size_t operator()(const tuple<int,Point>& k) const {
    size_t seed = 0;
    boost::hash_combine(seed, get<0>(k));
    boost::hash_combine(seed, get<1>(k).x);
    boost::hash_combine(seed, get<1>(k).y);
    return seed;
  }
};

class Crash {
  tsl::hopscotch_map<tuple<int,Point>, Robot *, int_point_hash>  pos;
  tsl::hopscotch_set<int> times;
  vector <Robot *> robots;

  void build(int t) {
    for(Robot *r : robots) {
      auto cur = make_tuple(t, r->pos(t));
      pos[cur] = r;
    }
    times.insert(t);
  }
  
public: 
  void add(Robot *r) {
    robots.push_back(r);
  }
  
  bool compatible(const Move &m) {
    if(!times.contains(m.t0))
      build(m.t0);
    if(!times.contains(m.t0+1))
      build(m.t0+1);

    auto dest_fut = make_tuple(m.t0+1, m.q);
    if(pos.count(dest_fut))
      return false; // Another robot will be at q at destination time

    auto dest_now = make_tuple(m.t0, m.q);
    if(pos.count(dest_now)) {
      Robot *r = pos.at(dest_now);
      if(!r->compatible(m))
        return false;
    }

    auto start_fut = make_tuple(m.t0+1, m.p);
    if(pos.count(start_fut)) {
      Robot *r = pos.at(start_fut);
      if(!r->compatible(m))
        return false;
    }
    
    return true;
  }
};

// Similar to crash but handles conflicting robots
class SuperCrash {
  tsl::hopscotch_map<tuple<int,Point>, vector<Robot *>, int_point_hash>  pos;
  tsl::hopscotch_set<int> times;
  vector <Robot *> robots;

  void build(int t) {
    for(Robot *r : robots) {
      auto cur = make_tuple(t, r->pos(t));
      pos[cur].push_back(r);
    }
    times.insert(t);
  }
  
public: 
  void add(Robot *r) {
    robots.push_back(r);
  }
  
  bool compatible(const Move &m) {
    if(!times.contains(m.t0))
      build(m.t0);
    if(!times.contains(m.t0+1))
      build(m.t0+1);

    auto dest_fut = make_tuple(m.t0+1, m.q);
    if(pos.count(dest_fut))
      return false; // Another robot will be at q at destination time

    auto dest_now = make_tuple(m.t0, m.q);
    if(pos.count(dest_now)) {
      for (Robot *r : pos.at(dest_now))
        if(!r->compatible(m))
          return false;
    }

    auto start_fut = make_tuple(m.t0+1, m.p);
    if(pos.count(start_fut)) {
      for (Robot *r : pos.at(start_fut))
        if(!r->compatible(m))
          return false;
    }
    
    return true;
  }
  
  tsl::hopscotch_set<Robot *> incompatible(const Move &m) {
    tsl::hopscotch_set<Robot *> ret;
    
    if(!times.contains(m.t0))
      build(m.t0);
    if(!times.contains(m.t0+1))
      build(m.t0+1);

    auto dest_fut = make_tuple(m.t0+1, m.q);
    if(pos.count(dest_fut))
      for(Robot *r : pos.at(dest_fut))
        ret.insert(r); // Another robot will be at q at destination time

    auto dest_now = make_tuple(m.t0, m.q);
    if(pos.count(dest_now)) {
      for(Robot *r : pos.at(dest_now))
        if(!r->compatible(m))
          ret.insert(r);
    }

    auto start_fut = make_tuple(m.t0+1, m.p);
    if(pos.count(start_fut)) {
      for(Robot *r : pos.at(start_fut))
        if(!r->compatible(m))
          ret.insert(r);
    }
    
    return ret;
  }
};


#endif
