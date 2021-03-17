// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Dichotomy solver
// Compile: g++ -std=c++20 -Ofast -o dichotomy dichotomy.cpp 
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
#include <optional>

#define VERB .5

int seed = -1;
#include "world.cpp"

using namespace std;

World dichotomySolver(World w, bool greedy=true) {
  if(!w.obstacles.empty()) {
    cout << "Obstacles not allowed!" << endl;
    exit(1);
  }
  
  cout <<  elapsedSec() << " Dichotomizing world" << endl;
  auto [minbox, maxbox] = boundingBox(w);
  int n = w.robots.size();

  Point center = minbox + maxbox;
  center.x /= 2;
  center.y /= 2;
  
  for(Robot *r : w.robots) {
    Point st = r->getStart();
    Point tg = r->getTarget();
    
    int dy = st.y - center.y;
    dy += (dy >= 0); // No line is zero
    
    // Top side
    if(st.y - center.y >= 0 && tg.x >= center.x)
      dy--; // Go up 1 line less for right target
      
    // Bottom side
    if(st.y - center.y < 0 && tg.x < center.x)
      dy++; // Go down 1 line less for left target
    
    int sdy = (dy > 0) - (dy < 0);
    
    // Move up or down according to bottom or to half
    r->reset();
    r->add(r->getStart());
    for(int i = 0; i < abs(dy); i++) {
      r->add(r->pos() + Point(0, sdy));
    }
  }    
   
  int txr[n];
  int width = maxbox.x - minbox.x;
  for(Robot *r : w.robots) {
    Point st = r->getStart();
    Point tg = r->getTarget();
    
    int tx = 1;
    
    for(Robot *r2 : w.robots) {
      if(r2->pos().y == r->pos().y) {
        if(tg.x < center.x && r->pos().x < r2->pos().x)
          tx++;
        else if(tg.x >= center.x && r->pos().x > r2->pos().x)
          tx++;
      }
    }
    
    if(r->pos().y >= minbox.y - 1 && r->pos().y <= maxbox.y + 1)
      tx += width / 2 + 1;

    if(tg.x < center.x)
      tx = - tx;
    else
      tx += width % 2;
    
    txr[r->getId()] = tx;
  }  

  for(Robot *r : w.robots) {
    int tx = txr[r->getId()];
    int stx = tx - (r->pos().x - center.x);
    stx = (stx > 0) - (stx < 0);

    r->add(r->pos());
    while(r->pos().x - center.x != tx) {
      r->add(r->pos() + Point(stx, 0));
    }
  }  
  
  if(!w.reversed)
    w.writeFile(basefn, "dichotomy", "network", true);
  
  cout << elapsedSec() << " Finding final paths" << endl;

  // Pre-sort by distance to target
  sort(w.robots.begin(), w.robots.end(), [](Robot *a, Robot *b){
    return a->nav->dist(a->pos()) > b->nav->dist(b->pos());});

  // Sort by horizontal center of target
  stable_sort(w.robots.begin(), w.robots.end(), [&center](Robot *a, Robot *b){
    return abs(a->getTarget().x - center.x) < abs(b->getTarget().x - center.x);});

  for(Robot *r : w.robots) {
//     cout << *r << endl;
    r->reset();  // Remove this line to see the paths to storage
    Randomner randomner;
    if(!w.findAstarPath(r, 2000, &randomner)) { // Use this line for random paths
      cout << *r << " could not find a path!" << endl;
      w.writeFile(basefn, "dichotomy", "failed");
    }

    assert(r->pos() == r->getTarget());
  }

  
  return w;
}


int main(int argc, char **argv) {
  if(seed < 0)
    seed = time(0)%1000000;
  srand(seed);

  cout << "STARTING " << argv[1] << endl;
  World w_original(argv[1]);

  w_original.meta["program"] = argv[0];
  w_original.meta["algorithm"] = "dichotomy";
  basefn = argv[1];
  basefn = basefn.substr(0, basefn.find_first_of(".")); // Remove extension

  for(int rot = 0; rot <= 1; rot++, w_original.rotate()) {
    cout << endl << "ROTATING:" << endl;
    for(int rev = 0; rev <= 1; rev++, w_original.reverse()) {
      if(rev) {
        cout << endl << "REVERSING:" << endl;
        if(elapsedSec() < 2)
          sleep(1); // To make sure I don't overwrite the file
      }
      World w = dichotomySolver(w_original);

      if(w.makespan() >= 0) {
        w.meta["computation_sec"] = to_string(elapsedSec());
        w.writeFile(basefn, "dichotomy", "solution");
        cout << "Execution took " << elapsedSec() << " seconds" << endl;
      }
    }
  }

  return 0;
}
