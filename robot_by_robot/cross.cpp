// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Cross solver
// Compile: g++ -std=c++20 -Ofast -o cross cross.cpp 
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
#include "tsl/hopscotch_map.h"
#include "tsl/hopscotch_set.h"

int seed = -1;
#include "world.cpp"

using namespace std;


World crossSolver(const World &w) {
  World crossWorld(w);
  for(Robot *r : crossWorld.robots) {
    r->add(r->getStart()); // Set robot as started
  }
  int n = crossWorld.robots.size();

  cout <<  elapsedSec() << " Building graph" << endl;
  auto [mininbox, maxinbox] = boundingBox(crossWorld);
  Point minoutbox, maxoutbox;
  
  minoutbox = mininbox - Point(3,3); // Add a frame of 3 to the bounding box
  maxoutbox = maxinbox + Point(3,3);

  Point p;
  auto depth = buildDepth(crossWorld.obstacles, minoutbox, maxoutbox);
  
  int radius = 1 + 3 * sqrt(n) / 4;
  Point mincross = minoutbox - Point(radius,radius);
  Point maxcross = maxoutbox + Point(radius,radius);

  vector<Point>  cross; // Holds the storage points
  // Horizontal arms of the cross
  for(p.x = mincross.x; p.x <= maxcross.x; p.x++) {
    for(p.y = minoutbox.y; p.y <= maxoutbox.y; p.y += 2) {
      if(!p.inside(minoutbox,maxoutbox)) {
        cross.push_back(p);
      }
    }
  }

  // Vertical arms of the cross
  for(p.x = minoutbox.x; p.x <= maxoutbox.x; p.x+=2) {
    for(p.y = mincross.y; p.y <= maxcross.y; p.y++) {
      if(!p.inside(minoutbox,maxoutbox)) {
        cross.push_back(p);
      }
    }
  }
 
  assert(cross.size() > n); // So every robot can be matched
  
  auto w_func = [](Robot *r, Point storage){return r->navrev->dist(storage) + r->nav->dist(storage);};

  cout << elapsedSec() << " Finding greedy matching" << endl;
  crossWorld.meta["algorithm"] = "greedyCross";
  vector<Point> copyCross(cross);
  // Sort by decreasing distance to target
  sort(crossWorld.robots.begin(), crossWorld.robots.end(),
        [](Robot *a, Robot *b){return a->nav->dist(a->pos()) > b->nav->dist(b->pos()); });

  for(Robot *r: crossWorld.robots) {
    int id = r->getId();
    Point st = r->getStart();
    Point tg = r->getTarget();
    
    int bestIndex = 0;
    Point bestTarget = copyCross[bestIndex];
    int bestWeight = w_func(r,bestTarget);
    for(int i = 1; i < copyCross.size(); i++) {
      Point storage = copyCross[i];
      int weight;
      weight = w_func(r, storage);
        
      if(weight < bestWeight) {
        bestTarget = storage;
        bestIndex = i;
        bestWeight = weight;
      }
    }
    r->setTarget(bestTarget);
    r->nav = new RotateNavigator(crossWorld.obstacles, bestTarget);
    copyCross.erase(copyCross.begin() + bestIndex);
    r->add(r->getStart());
  }

  cout << elapsedSec() << " Sending robots to cross" << endl;

  // Sort by increasing start depth breaking ties by point coordinate
  auto cmp = [&depth](Robot *a, Robot *b){
    return make_pair(depth[a->getStart()],a->getStart()) < make_pair(depth[b->getStart()],b->getStart());
  };
  sort(crossWorld.robots.begin(), crossWorld.robots.end(), cmp);

  // Find paths to cross for every robot in order
  for(Robot *r : crossWorld.robots) {
    Randomner randomner; 
    
//     if(!crossWorld.findAstarPath(r, 2000, &theValuer)) { // Use this line for deterministic paths
    if(!crossWorld.findAstarPath(r, 2000, &randomner)) { // Use this line for random paths
      cout << *r << " could not find a path to cross!" << endl;
      crossWorld.writeFile(basefn, "cross", "failed");
    }

    assert(r->pos() == r->getTarget());
  }

  if(!crossWorld.reversed)
    crossWorld.writeFile(basefn, "cross", "network", true);

  cout << elapsedSec() << " Finding final paths" << endl;

  // Restore crossWorld to the right destination
  for(Robot *r : crossWorld.robots) {
    delete r->nav;
    Robot *oldr = w.idmap.at(r->getId());
    r->setTarget(oldr->getTarget());
    r->nav = oldr->nav;
  }
 
  // Sort by decreasing distance from bounding box boundary to target
  auto cmp2 = [&depth](Robot *a, Robot *b){
    return make_tuple(depth[a->getTarget()], a->getTarget()) > make_tuple(depth[b->getTarget()], b->getTarget());
  };

  sort(crossWorld.robots.begin(), crossWorld.robots.end(), cmp2);

  for(Robot *r : crossWorld.robots) {
    r->reset();  // Remove this line to see the paths to storage
    Randomner randomner; 

//     if(!crossWorld.findAstarPath(r, 2000, &theValuer)) { // Use this line for deterministic paths
    if(!crossWorld.findAstarPath(r, 2000, &randomner)) { // Use this line for random paths
      cout << *r << " could not find a path!" << endl;
      crossWorld.writeFile(basefn, "cross", "failed");
    }

    assert(r->pos() == r->getTarget());
  }

  return crossWorld;
}


int main(int argc, char **argv) {
  if(seed < 0)
    seed = time(0)%1000000;
  srand(seed);

  cout << endl << "STARTING " << argv[1] << endl;
  World w_original(argv[1]);

  w_original.meta["program"] = argv[0];
  basefn = argv[1];
  basefn = basefn.substr(0, basefn.find_first_of(".")); // Remove extension

  for(int rev = 0; rev <= 1; rev++, w_original.reverse()) {
    if(rev) {
      cout << endl << "REVERSING:" << endl;
      if(elapsedSec() < 2)
        sleep(1); // To make sure I don't overwrite the file
    }
    World w = crossSolver(w_original);

    if(w.makespan() >= 0) {
      w.meta["computation_sec"] = to_string(elapsedSec());
      w.writeFile(basefn, "cross", "solution");
      cout << "Execution took " << elapsedSec() << " seconds" << endl;
    }
  }

  return 0;
}
