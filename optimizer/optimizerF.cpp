// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Feasible Optimzier
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
#include <cstdio>
#include <ctime>

#define VERB .5

int seed = -1;
#include "world.cpp"

using namespace std;

void optimizeRobot(World &w, Robot *r, Valuer *valuer, int r_wait = 0) {
  Robot oldr = *r;
  int r_mks = r->makespan();
  r->reset();
  
  for(int i = 0; i < r_wait; i++)
    r->add(r->getStart());

  w.findAstarPath(r, r_mks, valuer);
  assert(r->pos() == r->getTarget());
}

void optimize(World &w, bool late) {
  int max_mks = w.makespan();
  int n = w.robots.size();
  int maxvip = 2;
  vector<Robot *> vip;
  
  stable_sort(w.robots.begin(), w.robots.end(), [](Robot *a, Robot *b){return a->makespan() > b->makespan();});
  for(int i = 0; i < n && w.robots[i]->makespan() == max_mks; i++) {
    vip.push_back(w.robots[i]);
  }

  if(vip.size() > maxvip)
    vip.clear(); // Too many robots are vip, not worth it

  Stinky stinky(vip);
  bool beStinky = false;
  if(!vip.empty()) {
    if(rand() % 4 == 0)
      beStinky = true;

    if(vip.size() == 1)   
      cout << (beStinky ? "S" : "V")  << flush;
    else
      cout << (beStinky ? "s" : "v") << flush;
  }

  for(Robot *r : w.robots) {
      max_mks = w.makespan();
      int r_mks = r->makespan();
      
      Randomner randomner;
      Valuer *valuer = &randomner;
      if(beStinky)
        valuer = &stinky;
      
      if(late && r_mks < max_mks - 1 && rand() % 4 == 0) {
        w.reverse();
        int r_wait = 1 + rand() % (max_mks - r_mks - 1);
        optimizeRobot(w, r, valuer, r_wait);
        w.reverse();
      }
      else {
        optimizeRobot(w, r, valuer);      
      }
    
    for(Robot *r2 : vip) {
      if(r2->makespan() == max_mks) {
        Randomner randomner2;
        optimizeRobot(w, r2, &randomner2);
      }
    }
  }
}


void optimizeMany(World &w, World &best) {
  cout << w.makespan() << flush;
  
  int n = w.robots.size();
  int nopt = 64;

  if(n <= 400) {
    nopt *= 3;
  }
  if(n <= 200) {
    nopt *= 8;
  }
  if(n <= 150) {
    nopt *= 2;
  }

  for(int i = 0; i < nopt; i++) {
    int oldmks = w.makespan();

    optimize(w, i > nopt / 6);
    if(w.makespan() == oldmks) {
      if(w < best)
        cout << "-" << flush;
      else
        cout << "." << flush;
    }
    else {
      cout << "*" <<  w.makespan() << flush;
      i = 0;
    }

    if(updateBest(best,w)) {
      i -= 2; // A small bonus for reducing distance or number of mks robots
    }
  }
  cout << ">" <<  w.makespan() << endl;
}

int main(int argc, char **argv) {
  if(seed < 0)
    seed = time(0)%1000000;
  srand(seed);

  cout << endl << "STARTING " << argv[1] << endl;
  World w_original(argv[1]);
  if(w_original.betterReversed()) {
    w_original.reverse(); 
    cout << "Starting reversed" << endl;
  }

  w_original.meta["program"] = argv[0];
  basefn = argv[1];
  basefn = basefn.substr(0, basefn.find_first_of(".")); // Remove extension

  World best(w_original);
  World w(w_original);
  
  optimizeMany(w, best);

  best.meta["algorithm"] = "optimizerF";
  best.meta["computation_sec"] = to_string(elapsedSec());
  best.writeFile(basefn, "optimizerF", "solution");

  remove((basefn + ".gdf.tmp.sol.json").c_str());
  cout << "Execution took " << elapsedSec() << " seconds" << endl;
  

  return 0;
}
