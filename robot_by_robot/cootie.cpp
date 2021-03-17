// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Cootie Catcher solver
// Compile: g++ -std=c++20 -Ofast -o cootie cootie.cpp 
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

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdio>

#define VERB .5

int seed = -1;
#include "world.cpp"

using namespace std;


vector<Point> getDisplacements(Point p) {
  vector<Point> nw = {Point(0,1),  Point(-1,0), Point(1,0),  Point(0,-1), Point(0,0)};
  vector<Point> ne = {Point(0,1),  Point(1,0),  Point(-1,0), Point(0,-1), Point(0,0)};
  vector<Point> se = {Point(0,-1), Point(1,0),  Point(-1,0), Point(0,1),  Point(0,0)};
  vector<Point> sw = {Point(0,-1), Point(-1,0), Point(1,0),  Point(0,1),  Point(0,0)};  
  vector<Point> wn = {Point(-1,0), Point(0,1),  Point(1,0),  Point(0,-1), Point(0,0)};
  vector<Point> en = {Point(1,0),  Point(0,1),  Point(-1,0), Point(0,-1), Point(0,0)};
  vector<Point> es = {Point(1,0),  Point(0,-1), Point(-1,0), Point(0,1),  Point(0,0)};
  vector<Point> ws = {Point(-1,0), Point(0,-1), Point(1,0),  Point(0,1),  Point(0,0)};
  
  if(p.x <= 0 && p.y <= 0 && abs(p.x) > abs(p.y))
    return ws;
  if(p.x <= 0 && p.y <= 0 && abs(p.x) <= abs(p.y))
    return sw;
  
  if(p.x >= 0 && p.y >= 0 && abs(p.x) > abs(p.y))
    return en;
  if(p.x >= 0 && p.y >= 0 && abs(p.x) <= abs(p.y))
    return ne;
  
  if(p.x <= 0 && p.y >= 0 && abs(p.x) > abs(p.y))
    return wn;
  if(p.x <= 0 && p.y >= 0 && abs(p.x) <= abs(p.y))
    return nw;

  if(p.x >= 0 && p.y <= 0 && abs(p.x) > abs(p.y))
    return es;
  
//   if(p.x <= 0 && p.y >= 0 && abs(p.x) < abs(p.y))
  return se;
}

Point from(Point p, int w) {
  int x1=p.x,y1=p.y;

  if(p.x < 0 && abs(p.x) > abs(p.y)) {
    x1 = p.x - w/2 - 1;
    if(p.y > 0)
      y1 += (p.y)/2;
    if(p.y < 0)
      y1 += (p.y-1)/2;
    x1 += (abs(p.y))/2;
  }
  else if(p.x >= 0 && abs(p.x) > abs(p.y)) {
    x1 = p.x + w/2 + w%2;
    if(p.y > 0)
      y1 += (p.y)/2;
    if(p.y < 0)
      y1 += (p.y-1)/2;
    x1 -= (abs(p.y))/2;
  }
  else if(p.y < 0) {
    y1 = p.y - w/2 - 1;
    if(p.x > 0)
      x1 += (p.x)/2;
    if(p.x < 0)
      x1 += (p.x-1)/2;
    y1 += (abs(p.x))/2;
  }
  else {
    y1 = p.y + w/2 + 1 + w%2;
    if(p.x > 0)
      x1 += (p.x)/2;
    if(p.x < 0)
      x1 += (p.x-1)/2;
    y1 -= (abs(p.x))/2;
  }

  return Point(x1,y1);
}


World cootieSolver(const World &w, bool greedy=true) {
  cout <<  elapsedSec() << " Building cootie world" << endl;
  auto [minbox, maxbox] = boundingBox(w); 
  
  Point v_0((minbox.x+maxbox.x+1)/2, (minbox.y+maxbox.y+1)/2);
  
  int n = w.robots.size();
  int width = max(maxbox.x-minbox.x, maxbox.y- minbox.y) + 1;

  World cootieWorld(w);
  for(Robot *r : cootieWorld.robots) {
    Point st = r->getStart();
    r->add(st); // Set robot as started
    Point tg = from(st-v_0, width) + v_0;
    r->setTarget(tg);
    r->nav = new RotateNavigator(cootieWorld.obstacles, tg);
  }

  cout << elapsedSec() << " Sending robots outside" << endl;

  Point p;
  auto depth = buildDepth(cootieWorld.obstacles, minbox, maxbox);

  // Sort by increasing start depth breaking ties by distance to storage
  auto cmp = [&depth](Robot *a, Robot *b){
    return make_pair(depth[a->getStart()],-a->makespanLB()) < make_pair(depth[b->getStart()],-b->makespanLB());
  };

  stable_sort(cootieWorld.robots.begin(), cootieWorld.robots.end(), cmp);

  // Find paths to cootie for every robot in order
  for(Robot *r : cootieWorld.robots) {
    auto displacements = getDisplacements(r->getStart() - v_0);
    
    if(!cootieWorld.findAstarPath(r, 2000, &theValuer, displacements)) {
      cout << *r << " could not find a path to cootie!" << endl;
      cootieWorld.writeFile(basefn, "cootie", "failed");
    }

    assert(r->pos() == r->getTarget());
  }

  if(!cootieWorld.reversed)
    cootieWorld.writeFile(basefn, "cootie", "network", true);

  cout << elapsedSec() << " Finding final paths" << endl;

  for(Robot *r : cootieWorld.robots) {
    delete r->nav;
    Robot *oldr = w.idmap.at(r->getId());
    r->nav = oldr->nav;
    r->setTarget(oldr->getTarget());
  }

 // Sort by decreasing distance from bounding box boundary to target
  auto cmp2 = [&depth](Robot *a, Robot *b){
    return make_tuple(depth[a->getTarget()], a->makespanLB()) > make_tuple(depth[b->getTarget()], b->makespanLB());
  };

  stable_sort(cootieWorld.robots.begin(), cootieWorld.robots.end(), cmp2);

//--------------------------------
  // Find paths to cross for every robot in order
  for(Robot *r : cootieWorld.robots) {
//     cout << *r << endl;
    Randomner randomner;
    r->reset();  // Remove this line to see the paths to storage
  
    Crash crash;
    for(Robot *r2 : cootieWorld.robots) {
      if(r2->started() && r2 != r)
        crash.add(r2);
    }
    
//     if(!cootieWorld.findAstarPath(r, 2000, &theValuer)) { // Use this line for deterministic paths
    if(!cootieWorld.findAstarPath(r, 2000, &randomner)) { // Use this line for random paths
      cout << *r << " could not find a path to inside!" << endl;
      cootieWorld.writeFile(basefn, "cootie", "failed");
      exit(1);
    }

    assert(r->pos() == r->getTarget());
  }
  
  return cootieWorld;
}


int main(int argc, char **argv) {
  if(seed < 0)
    seed = time(0)%1000000;
  srand(seed);

  cout << "STARTING " << argv[1] << endl;
  World w_original(argv[1]);

  w_original.meta["program"] = argv[0];
  basefn = argv[1];
  basefn = basefn.substr(0, basefn.find_first_of(".")); // Remove extension

  for(int rev = 0; rev < 2; rev++, w_original.reverse()) {
    if(rev) {
      cout << endl << "REVERSING:" << endl;
      if(elapsedSec() < 2)
        sleep(1); // To make sure I don't overwrite the file
    }
    World w = cootieSolver(w_original);

    if(w.makespan() >= 0) {
      w.meta["computation_sec"] = to_string(elapsedSec());
      w.writeFile(basefn, "cootie", "solution");
      cout << "Execution took " << elapsedSec() << " seconds" << endl;
    }
  }

  return 0;
}
