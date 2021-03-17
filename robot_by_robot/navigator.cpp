// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Class to find distances around obstacles
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
#ifndef NAVIGATOR
#define NAVIGATOR

#include <iostream>
#include <functional>
#include <string>
#include <queue>
#include <vector>
#include "tsl/hopscotch_set.h"
#include "tsl/hopscotch_map.h"

#include "primitives.cpp"

using namespace std;

class Navigator {
protected:
  Point minxy, maxxy;
  vector<vector<pair<int,int>>> compressed;

  tsl::hopscotch_map<Point,int> bfs(const tsl::hopscotch_set<Point> &obstacles, Point target) {
    const vector<Point> displacements{{Point(0,1), Point(0,-1), Point(1,0), Point(-1,0)}};
    tsl::hopscotch_map<Point,int> dists;
    queue<Point> fifo;
    fifo.push(target);
    dists[target] = 0;

    while(!fifo.empty()) {
      Point p = fifo.front();
      int dp = dists[p];
      fifo.pop();
      for(Point v : displacements) {
        Point q = p + v;
        if(q.x >= minxy.x && q.y >= minxy.y && q.x <= maxxy.x && q.y <= maxxy.y &&
           !obstacles.contains(q) && !dists.contains(q)) {
          fifo.push(q);
          dists[q] = dp + 1;
        }
      }
    }
    return dists;
  }

  void compress(const tsl::hopscotch_map<Point,int> &dists) {
    Point cur;

    for(cur.y = minxy.y; cur.y <= maxxy.y; cur.y++) {
      compressed.push_back(vector<pair<int,int>>());
      for(cur.x = minxy.x; cur.x <= maxxy.x; cur.x++) {
        Point left = cur - Point(1,0);
        Point right = cur + Point(1,0);
        if(dists.contains(cur)) {
          if(!dists.contains(left) ||
            !dists.contains(right) ||
            dists.at(cur) != (dists.at(left) + dists.at(right)) / 2) {
            compressed.back().push_back(make_pair((int)cur.x, dists.at(cur)));
          }
        }
      }
    }
  }

public:
  Navigator(const tsl::hopscotch_set<Point> obstacles, Point target) {
    if(obstacles.empty()) {
      minxy = maxxy = target;
      return;
    }

    minxy = maxxy = target;
    for(Point p : obstacles) {
      minxy.x = min(p.x, minxy.x);
      minxy.y = min(p.y, minxy.y);
      maxxy.x = max(p.x, maxxy.x);
      maxxy.y = max(p.y, maxxy.y);
    }
    minxy = minxy - Point(1,1); // Add a frame of 1
    maxxy = maxxy + Point(1,1);

    tsl::hopscotch_map<Point,int> dists = bfs(obstacles, target);
    compress(dists);
  }

  int dist(Point q) const {
    if(minxy == maxxy) // No obstacles!
      return minxy.l1(q);

    int ret = 0;
    int line;
    if(q.y < minxy.y) {
      line = 0;
      ret += minxy.y - q.y;
    }
    else if(q.y > maxxy.y) {
      line = compressed.size() - 1;
      ret += q.y - maxxy.y;
    }
    else {
      line = q.y - minxy.y;
    }

    auto line_begin = compressed[line].begin();
    auto line_end = compressed[line].end();
    auto it = upper_bound(line_begin, line_end, make_pair((int)q.x,-1));

    int next_x, next_dist = -1;
    if(it != line_end) {
      next_x = it->first;
      next_dist = it->second;
      if(next_x == q.x)
          return ret + next_dist;
    }

    int prev_x, prev_dist = -1;
    if(it != line_begin) {
      --it;
      prev_x = it->first;
      prev_dist = it->second;
    }

    if(prev_dist == -1) { // Extrapolate
      ret += next_dist + next_x - q.x;
      return ret;
    }

    if(next_dist == -1) { // Extrapolate
      ret += prev_dist + q.x - prev_x;
      return ret;
    }

    // Interpolate
    ret += prev_dist + (next_dist - prev_dist) * (q.x - prev_x) / (next_x - prev_x);
    return ret;
  }
};

class RotateNavigator : public Navigator {
protected:
  int rot = 0;
public:
  using Navigator::Navigator;

  void rotate(int r = 1) {
    rot += r;
    while(rot < 0)
      rot += 4;
    rot %= 4;
  }

  int dist(Point p) {
    p.rotate(-rot);
    return Navigator::dist(p);
  }
};
#endif
