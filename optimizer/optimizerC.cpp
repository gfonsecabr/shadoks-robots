// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Conflict Optimizer
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


bool optimizeConflicts(World &w) {
  int max_mks = w.makespan();
  int n = w.robots.size();
//   cout << "Robots with makespan " << max_mks << ": ";
  tsl::hopscotch_set<Robot *> todo_set;
  queue<Robot *> todo_queue;
  tsl::hopscotch_map<Robot *, int> done;
  
  stable_sort(w.robots.begin(), w.robots.end(), [](Robot *a, Robot *b){return a->makespan() > b->makespan();});
  for(int i = 0; i < n && w.robots[i]->makespan() == max_mks; i++) {
    Robot *r = w.robots[i];
    todo_queue.push(r);
    todo_set.insert(r);
  }

  int maxdone = 1;

  while(!todo_queue.empty()) {
    
    Robot *r = todo_queue.front();
    todo_queue.pop();
    todo_set.erase(r);
    
    r->reset();
    done[r]++;
    if(done[r] > 64) {
      cout << endl << "We rerouted too many times " << *r << endl;
      cout << "!" << flush;
      return false;
    }
    
    if(done[r] > maxdone) {
      maxdone = done[r];
      cout << "maxdone: " << maxdone << " todo: " << todo_set.size() << endl;
    }

//     Randomner randomner; // For random paths
//     tsl::hopscotch_set<Robot *> conflicts = w.findIllegalPath(r, max_mks - 1, &randomner, done);
    tsl::hopscotch_set<Robot *> conflicts = w.findIllegalPath(r, max_mks - 1, &theValuer, done); // For deterministic paths
    vector<Robot *> conflicts_v(conflicts.begin(), conflicts.end());
    random_shuffle(conflicts_v.begin(), conflicts_v.end());
    for(Robot *rc : conflicts_v) {
      if(!todo_set.contains(rc)) {
        todo_queue.push(rc);
        todo_set.insert(rc);
      }
    }
  }
  
  cout << endl << w.makespan();
  return true;
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
  cout << best.makespan() << flush;
  
  while(best.makespan() != best.makespanLB()) {
    World w_previous(best);
    
    if(optimizeConflicts(best)) {
      best.meta["algorithm"] = "optimizerC";
      best.meta["computation_sec"] = to_string(elapsedSec());
      best.writeFile(basefn, "optimizerC", "solution");
    }
    else {
      cout << "This line is never reached" << endl;
    }
  }

  cout << "Execution took " << elapsedSec() << " seconds" << endl;

  return 0;
}
