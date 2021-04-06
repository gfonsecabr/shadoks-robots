// CG:SHOP 2021 Coordinated Motion Planning - Shadoks Team
// 
// Class that stores all robots and obstacles together
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
#ifndef WORLD
#define WORLD

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <stack>
#include <filesystem>
#include <unistd.h>
#include <string.h>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/ostreamwrapper.h"
#include "tsl/hopscotch_map.h"
#include "tsl/hopscotch_set.h"
#include "robot.cpp"
#include "crash.cpp"
#include "navigator.cpp"

using namespace std;

class World {
  void copy_attributes(const World &w) {
    for(const Robot *r : w.robots) {
      Robot *r2 = new Robot(*r);
      add_robot(r2);
    }

    obstacles = w.obstacles;
    meta = w.meta;
    rotation = w.rotation;
    reversed = w.reversed;
  }


  rapidjson::Document readJson(string filename) {
    ifstream in(filename, ifstream::in | ifstream::binary);
    if (!in.is_open())
    {
      cerr << "Error reading " << filename << endl;
      exit(EXIT_FAILURE);
    }

    rapidjson::IStreamWrapper isw {in};

    rapidjson::Document doc {};
    doc.ParseStream(isw);

    if (doc.HasParseError())
    {
      cerr << "Error  : " << doc.GetParseError()  << endl;
      cerr << "Offset : " << doc.GetErrorOffset() << endl;
      exit(EXIT_FAILURE);
    }
    return doc;
  }

  void readInstance(rapidjson::Document &doc) {
    const rapidjson::Value& json_obstacles = doc["obstacles"];
    for (auto &ob : json_obstacles.GetArray()) {
      assert(ob.IsArray());
      const int x = ob[0].GetInt();
      const int y = ob[1].GetInt();
      add_obstacle(x, y);
    }

    const rapidjson::Value& json_targets = doc["targets"];
    int id = 0;
    for (auto &ob : json_targets.GetArray()) {
      assert(ob.IsArray());
      Point p;
      p.x = ob[0].GetInt();
      p.y = ob[1].GetInt();

      Robot *r = new Robot(p, p, id);
      add_robot(r);
      r->nav = new RotateNavigator(obstacles, r->getTarget());

      id++;
    }

    const rapidjson::Value& json_starts = doc["starts"];
    id = 0;
    for (auto &ob : json_starts.GetArray()) {
      assert(ob.IsArray());
      Point p;
      p.x = ob[0].GetInt();
      p.y = ob[1].GetInt();

      Robot *r = idmap[id];
      r->setStart(p);
      r->navrev = new RotateNavigator(obstacles, r->getStart());

      id++;
    }
    
    meta["instance"] = doc["name"].GetString();
  }
  
  void readSolution(rapidjson::Document &doc) {
    const rapidjson::Value& json_steps = doc["steps"];
    for(Robot *r : robots) {
      r->add(r->getStart());
    }
    
    for (auto &moves : json_steps.GetArray()) {
      for(Robot *r : robots) {
        Point v = Point(0,0);
        string id = to_string(r->getId());
        
        if(moves.HasMember(id.c_str())) {
          string smove = moves[id.c_str()].GetString();
          switch(smove[0]) {
            case 'N': v = Point(0,+1); break;
            case 'S': v = Point(0,-1); break;
            case 'E': v = Point(+1,0); break;
            case 'W': v = Point(-1,0); break;
          }
        }
        r->add(r->pos() + v);
      }
    }
    
    meta["input_makespan"] = to_string(makespan());
    meta["input_distance"] = to_string(distance());
  }
  
  
public:
  vector<Robot *> robots;
  tsl::hopscotch_set<Point> obstacles;
  tsl::hopscotch_map<int, Robot *> idmap;
  map<string, string> meta;
  int rotation = 0;
  bool reversed = false;

  World() {
  }

  World(string filename) {
    rapidjson::Document doc = readJson(filename);
    if(doc.HasMember("starts")) {
      readInstance(doc);
    }
    else { // File is a solution
      string inst_name = doc["instance"].GetString();
      std::filesystem::path p = filename;
      p.remove_filename();
      rapidjson::Document doc2 = readJson(p.string() + inst_name + ".instance.json");
      readInstance(doc2);
      readSolution(doc);
    }
    
    meta["input"] = filename;
    meta["author"] = "gdf";
    meta["start_time"] = timeString();
    char hn[80];
    gethostname(hn, 80);
    meta["host"] = string(hn);
  }

  World(const World &w) {
    copy_attributes(w);
  }

  World& operator = (const World &w) {
    for(Robot *r : robots)
      delete r;

    robots.clear();
    idmap.clear();

    copy_attributes(w);

    return *this;
  }

  bool operator< (const World &w) const {
    if(w.robots.size() == 0 && robots.size() > 0)
      return true;
    return make_tuple(makespan(), countMakespan(), distance()) < make_tuple(w.makespan(), w.countMakespan(), w.distance());
  }

  ~World() {
    for(Robot *r : robots)
      delete r;
  }

  void rotate(int rot = 1) {
    tsl::hopscotch_set<Point> old(obstacles);
    obstacles.clear();
    for(Point p : old) {
      p.rotate(rot);
      obstacles.insert(p);
    }
    for(Robot *r : robots) {
      r->rotate(rot);
    }

    rotation += rot;
    while(rotation < 0)
      rotation += 4;
    rotation %= 4;
  }

  void reverse() {
    reversed = !reversed;
    unifyMakespan();
    for(Robot *r : robots) {
      r->reverse();
    }
  }

  void reset() {
    for(Robot *r : robots) {
      r->reset();
    }
  }

  void add_robot(Robot *r) {
    robots.push_back(r);
    idmap[r->getId()] = r;
  }

  void add_obstacle(int x, int y) {
    Point p(x,y);
    obstacles.insert(p);
  }

  int makespan() const {
    int x = -1;
    for(Robot *r : robots) {
      if(r->makespan() > x)
        x = r->makespan();
    }

    return x;
  }

  int countMakespan() const {
    int mks = makespan();
    int x = 0;
    for(Robot *r : robots) {
      if(r->makespan() == mks)
        x++;
    }

    return x;
  }
  
  int distance() const {
    int x = 0;
    for(Robot *r : robots) {
      x += r->distance();
    }

    return x;
  }

  int makespanLB() const {
    int x = 0;
    for(Robot *r : robots) {
      if(r->makespanLB() > x)
        x = r->makespanLB();
    }

    return x;
  }

  int distanceLB() const {
    int x = 0;
    for(Robot *r : robots) {
      x += r->makespanLB();
    }

    return x;
  }

  bool canStay(Robot *r, int t) const {
    int mks = makespan();
    Point target = r->getTarget();

    for(Robot *r2 : robots) {
      if(r2->started() && r != r2) {
        for(int i = t; i <= r2->time(); i++) {
          if(!r2->compatible(Move(target,target,i)))
            return false;
        }
      }
    }
    return true;
  }

  bool findAstarPath(Robot *r, int limitastar, Valuer *valuer) {
    const vector<Point> displacements{{Point(0,0), Point(0,1), Point(0,-1), Point(1,0), Point(-1,0)}};

    Crash crash;
    for(Robot *r2 : robots) {
      if(r2->started() && r2 != r)
        crash.add(r2);
    }

    struct Node {
      int done, todo;
      double distdone;
      int disttodo;
      Move m;

      int cost() const {
        return done + todo;
      }

      double distcost() const{
        return distdone + disttodo;
      }

      auto operator<=>(const Node& v) const {
        return make_tuple(  cost(),   distcost(),   todo,   disttodo,   m)
           <=> make_tuple(v.cost(), v.distcost(), v.todo, v.disttodo, v.m);
      }

      string toString() const {
        string s;
        s = "[" + to_string(cost()) + "," + to_string(distcost())
          + "=" + to_string(done) + + "," + to_string(distdone)
          + "+" + to_string(todo) + "," + to_string(disttodo)
          + ";" + m.toString() + "]";

        return s;
      }
    };

    vector<Node> heap;
    tsl::hopscotch_map<tuple<int,Point>, Node, int_point_hash> dealt;
    tsl::hopscotch_map<Move,Move> previous;

    Node node0;
    node0.done = 0;
    node0.distdone = 0;
    node0.todo = r->nav->dist(r->pos());
    node0.disttodo = node0.todo;
    node0.m = Move(r->pos(), r->pos(), r->time() );
    previous[node0.m] = node0.m; // Root of the tree points to itself

    heap.push_back(node0);

    while(!heap.empty()) {
      Node mini = heap[0];

      pop_heap(heap.begin(), heap.end(), std::greater<>{});
      heap.pop_back();

      if(mini.m.q == r->getTarget() && canStay(r, mini.m.t0 + 1)) {
        // We are done with this path!
        stack<Point> reverse_path;
        Move m = mini.m;
        reverse_path.push(m.q);
        while(previous[m] != m) {
          m = previous[m];
          reverse_path.push(m.q);
        }

        while(!reverse_path.empty()) {
          Point p = reverse_path.top();
          reverse_path.pop();
          r->add(p);
        }

        return true;
      }

      // Add other nodes
      for(Point v : displacements) {
        Point q2 = mini.m.q + v;
        Move m2 = Move(mini.m.q, q2, mini.m.t0 + 1);
        auto tup = make_tuple(m2.t0, q2);
        if(!obstacles.contains(q2) && crash.compatible(m2)) {
          Node node;
          node.done = m2.t0 + 1;
          node.todo = r->nav->dist(q2);
          node.m = m2;
          node.distdone = mini.distdone + v.l1() * valuer->get(q2, m2.t0 + 1);
          node.disttodo = node.todo;

          if(node.cost() <= limitastar && (!dealt.contains(tup) || dealt[tup] > node)) {
            dealt[tup] = node;
            heap.push_back(node);
            push_heap(heap.begin(), heap.end(), std::greater<>{});

            previous[m2] = mini.m;
          }
        }
//         else {  // incompatible move is not added
//         }
      }
    }

    return false;
  }

  
  // This version keeps the dispacements in order
  bool findAstarPath(Robot *r, int limitastar, Valuer *valuer, vector<Point> displacements) {
    Crash crash;
    for(Robot *r2 : robots) {
      if(r2->started() && r2 != r)
        crash.add(r2);
    }

    struct Node {
      int done, todo;
      double distdone;
      int disttodo;
      int moveidx;
      Move m;

      int cost() const {
        return done + todo;
      }

      double distcost() const{
        return distdone + disttodo;
      }

      auto operator<=>(const Node& v) const {
        return make_tuple(  cost(),   distcost(),   todo,   disttodo, moveidx,     m)
           <=> make_tuple(v.cost(), v.distcost(), v.todo, v.disttodo, v.moveidx, v.m);
      }

      string toString() const {
        string s;
        s = "[" + to_string(cost()) + "," + to_string(distcost())
          + "=" + to_string(done) + + "," + to_string(distdone)
          + "+" + to_string(todo) + "," + to_string(disttodo)
          + ";" + m.toString() + "]";

        return s;
      }
    };

    vector<Node> heap;
    tsl::hopscotch_map<tuple<int,Point>, Node, int_point_hash> dealt;
    tsl::hopscotch_map<Move,Move> previous;

    Node node0;
    node0.done = 0;
    node0.distdone = 0;
    node0.todo = r->nav->dist(r->pos());
    node0.disttodo = node0.todo;
    node0.m = Move(r->pos(), r->pos(), r->time() );
    previous[node0.m] = node0.m; // Root of the tree points to itself

    heap.push_back(node0);

    while(!heap.empty()) {
      Node mini = heap[0];

      pop_heap(heap.begin(), heap.end(), std::greater<>{});
      heap.pop_back();

      if(mini.m.q == r->getTarget() && canStay(r, mini.m.t0 + 1)) {
        // We are done with this path!
        stack<Point> reverse_path;
        Move m = mini.m;
        reverse_path.push(m.q);
        while(previous[m] != m) {
          m = previous[m];
          reverse_path.push(m.q);
        }

        while(!reverse_path.empty()) {
          Point p = reverse_path.top();
          reverse_path.pop();
          r->add(p);
        }

        return true;
      }

      // Add other nodes
      for(int movei = 0; movei < displacements.size(); movei++) {
        Point v = displacements[movei];
        Point q2 = mini.m.q + v;
        Move m2 = Move(mini.m.q, q2, mini.m.t0 + 1);
        auto tup = make_tuple(m2.t0, q2);
        if(!obstacles.contains(q2) && crash.compatible(m2)) {
          Node node;
          node.done = m2.t0 + 1;
          node.todo = r->nav->dist(q2);
          node.m = m2;
          node.distdone = mini.distdone + v.l1() * valuer->get(q2, m2.t0 + 1);
          node.disttodo = node.todo;
          node.moveidx = movei;

          if(node.cost() <= limitastar && (!dealt.contains(tup) || dealt[tup] > node)) {
            dealt[tup] = node;
            heap.push_back(node);
            push_heap(heap.begin(), heap.end(), std::greater<>{});

            previous[m2] = mini.m;
          }
        }
      }
    }

    return false;
  }
  
  
  tsl::hopscotch_set<Robot *> findIllegalPath(Robot *r, int limitastar, Valuer *valuer, tsl::hopscotch_map<Robot *, int> &conflict_cost) {
    const vector<Point> displacements{{Point(0,0), Point(0,1), Point(0,-1), Point(1,0), Point(-1,0)}};

    SuperCrash crash;
    for(Robot *r2 : robots) {
      if(r2->started() && r2 != r)
        crash.add(r2);
    }

    struct Node {
      int done, todo;
      double distdone;
      int disttodo;
      tsl::hopscotch_set<Robot *> conflicts;
      Move m;
      tsl::hopscotch_map<Robot *, int> *conflict_cost;

      int cost() const {
        return done + todo;
      }

      double distcost() const{
        return distdone + disttodo;
      }

      auto operator<=>(const Node& v) const {
        int conf_cost1 = 0, conf_cost2 = 0;
        for(Robot *r : conflicts) {
          conf_cost1++;
          if(conflict_cost->contains(r))
            conf_cost1 += conflict_cost->at(r) * conflict_cost->at(r);
        }

        for(Robot *r : v.conflicts) {
          conf_cost2++;
          if(conflict_cost->contains(r))
            conf_cost2 += conflict_cost->at(r) * conflict_cost->at(r);
        }
          
        return make_tuple(conf_cost1,   cost(),   distcost(),   todo,   disttodo,   m)
           <=> make_tuple(conf_cost2, v.cost(), v.distcost(), v.todo, v.disttodo, v.m);
      }

      string toString() const {
        string s;
        s = "[" + to_string(cost()) + "," + to_string(distcost())
          + "=" + to_string(done) + + "," + to_string(distdone)
          + "+" + to_string(todo) + "," + to_string(disttodo)
          + ";" + m.toString() + "]";

        return s;
      }
    };

    vector<Node> heap;
    tsl::hopscotch_map<tuple<int,Point>, Node, int_point_hash> dealt;
    tsl::hopscotch_map<Move,Move> previous;

    Node node0;
    node0.done = 0;
    node0.distdone = 0;
    node0.todo = r->nav->dist(r->pos());
    node0.disttodo = node0.todo;
    node0.m = Move(r->pos(), r->pos(), r->time() );
    previous[node0.m] = node0.m; // Root of the tree points to itself
    node0.conflict_cost = &conflict_cost;

    heap.push_back(node0);

    while(!heap.empty()) {
      Node mini = heap[0];

      pop_heap(heap.begin(), heap.end(), std::greater<>{});
      heap.pop_back();

      if(mini.m.q == r->getTarget() && mini.m.t0 + 1 == limitastar) {
        // We are done with this path!
        stack<Point> reverse_path;
        Move m = mini.m;
        reverse_path.push(m.q);
        while(previous[m] != m) {
          m = previous[m];
          reverse_path.push(m.q);
        }

        while(!reverse_path.empty()) {
          Point p = reverse_path.top();
          reverse_path.pop();
          r->add(p);
        }
       
        return mini.conflicts;
      }

      // Add other nodes
      for(Point v : displacements) {
        Point q2 = mini.m.q + v;
        Move m2 = Move(mini.m.q, q2, mini.m.t0 + 1);
        auto tup = make_tuple(m2.t0, q2);
        if(!obstacles.contains(q2)) {
          Node node;
          node.done = m2.t0 + 1;
          node.todo = r->nav->dist(q2);
          node.m = m2;
          node.distdone = mini.distdone + v.l1() * valuer->get(q2, m2.t0 + 1);
          node.disttodo = node.todo;
          node.conflicts = mini.conflicts;
          node.conflict_cost = &conflict_cost;
          for(Robot *rc : crash.incompatible(m2)) {
            node.conflicts.insert(rc);
          }
          
          if(node.cost() <= limitastar && (!dealt.contains(tup) || dealt[tup] > node)) {
            dealt[tup] = node;
            heap.push_back(node);
            push_heap(heap.begin(), heap.end(), std::greater<>{});

            previous[m2] = mini.m;
          }
        }
      }
    }

    return {r};
  }

  void unifyMakespan() {
    int mks = makespan();
    if(mks > 0) {
      for (Robot *r : robots) {
        r->uniformPath(mks);
      }
    }
  }

  bool betterReversed() {
    int countfw=0, countrev=0;
    int mks = makespan();
    for(Robot *r : robots) {
      if(r->pos(0) != r->pos(1))
        countrev++;
      if(r->pos(mks) != r->pos(mks-1))
        countfw++;
    }
    
    cout << "Best mks robots " << countfw << ", reversed " << countrev << endl;
    
    return countrev < countfw;
  }
  
  void writeFile(const string &basefn, const string &alg, const string &sols, bool quiet = false) const
  {
    string tstr = timeString();
    string filename = basefn + "." + alg + "." + tstr + "." + sols + ".json";

    World w(*this);
    // To forbid rotationg the navigators
    for(Robot *r : w.robots)
      r->nav = r->navrev = NULL;
    w.rotate(-rotation);
    if(reversed)
      w.reverse();
    w.unifyMakespan();
    w.meta["makespan"] = to_string(makespan());
    w.meta["distance"] = to_string(distance());
    w.meta["save_time"] = timeString();
    w.meta["rotation"] = to_string(rotation);
    w.meta["reversed"] = to_string(reversed);
    w.meta["seed"] = to_string(seed);
    sort(w.robots.begin(), w.robots.end(), [](Robot *a, Robot *b){return a->getId() < b->getId();});

    if(!quiet)
      print("Saving to " + filename);

    ofstream file(filename, fstream::out | ifstream::binary);

    file << "{" << endl;
    file << "\t\"instance\": \"" << w.meta["instance"] << "\"," << endl;

    file << "\t\"meta\": {" << endl;
    for(auto const& [key, val] : w.meta) {
      file << "\t\t\"" << key << "\": \"" << val << "\"," << endl;
    }
    file << "\t\t\"" << "last_meta\": \"\"" << endl;
    file << "\t}," << endl;

    file << "\t\"steps\": [" << endl;

    int tmax = w.makespan();
    for (int t = 1; t <= tmax; t++) {
      string line = "\t\t{";
      for (Robot *r : w.robots) {
        Point v = r->pos(t) - r->pos(t-1);
        if(v != Point(0,0)) {
          line += "\"" + to_string(r->getId()) + "\": \"";
          if(v == Point(0,1))
            line += "N";
          else if(v == Point(0,-1))
            line += "S";
          else if(v == Point(1,0))
            line += "E";
          else if(v == Point(-1,0))
            line += "W";
          else {
            line += "N";
            cout << "INVALID MOVE REPLACED WITH NORTH!" << endl;
          }
          
            
          line += "\", ";
        }
      }
      if(line.back() == ' ') {
        line.pop_back(); // Remove space
        line.pop_back(); // Remove commma
      }
      line += "}";
      if (t != tmax)
        line += ",";
      file << line << endl;
    }
    file << "\t]" << endl;
    file << "}" << endl;
    file.close();
  }

  void print(const string &s) const {
    cout << s <<
      " mks: " << makespan() << " / " << makespanLB() <<
      " dist: " << distance() << " / " << distanceLB() << endl;
  }
};

string basefn;

bool updateBest(World &best, const World &w) {
  if(w < best) {
    best = w;
    w.writeFile(basefn, "tmp", "solution", true);
    return true;
  }
  return false;
}

tsl::hopscotch_map<Point, int> buildDepth(const tsl::hopscotch_set<Point> &obstacles, Point minbox, Point maxbox) {
  tsl::hopscotch_map<Point, int> depth;
  vector<Point> boundary;
  Point p;
  for(p = minbox; p.x < maxbox.x; p.x++) {
    boundary.push_back(p); // Bottom edge
    boundary.push_back(Point(p.x, maxbox.y)); // Top edge
  }
  for(p = minbox; p.y < maxbox.y; p.y++) {
    boundary.push_back(p); // Left edge
    boundary.push_back(Point(maxbox.x, p.y)); // Right edge
  }
  
  for(Point q : boundary) {
    Navigator nav(obstacles, q);
    for(p.x = minbox.x; p.x <= maxbox.x; p.x++) {
      for(p.y = minbox.y; p.y <= maxbox.y; p.y++) {
        if(depth.contains(p))
          depth[p] = min(nav.dist(p), depth[p]);
        else
          depth[p] = nav.dist(p);
      }
    }
  }
  
  return depth;
}

pair<Point,Point> boundingBox(const World &w) {
  Point minxy, maxxy;

  minxy = maxxy = w.robots[0]->getStart();

  for(Robot *r : w.robots) {
    for(Point p : {r->getStart(), r->getTarget()}) {
      minxy.x = min(p.x, minxy.x);
      minxy.y = min(p.y, minxy.y);
      maxxy.x = max(p.x, maxxy.x);
      maxxy.y = max(p.y, maxxy.y);
    }
  }

  for(Point p : w.obstacles) {
      minxy.x = min(p.x, minxy.x);
      minxy.y = min(p.y, minxy.y);
      maxxy.x = max(p.x, maxxy.x);
      maxxy.y = max(p.y, maxxy.y);
  }

  return {minxy,maxxy};
}

#endif
