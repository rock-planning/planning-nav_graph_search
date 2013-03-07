/* Dstar.h
 * James Neufeld (neufeld@cs.ualberta.ca)
 * v1.0 modified by Stefan Haase (stefan.haase@dfki.de)
 */

#ifndef DSTAR_H
#define DSTAR_H

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <ext/hash_map>

namespace dstar_lite {

class state {
 public:

  state() {
    x = 0;
    y = 0;
  }

  state(int x, int y) {
    this->x = x;
    this->y = y;
  }

  int x;
  int y;
  std::pair<double,double> k; // Priority of the state (vertex) within the priority queue.
  
  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }
  
  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }
  
  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;

  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;

  }
  

  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;

  }
   
};

struct ipoint2 {
 public:
  ipoint2() {
    x = 0;
    y = 0;
  }

  ipoint2(int x, int y) {
    this->x = x;
    this->y = y;
  }

  bool operator < (const ipoint2 &ip2) const {
    return(x < ip2.x || (x == ip2.x && y < ip2.y));
  }

  int x,y;
};

struct cellInfo {
 public:
  cellInfo() {
    g = 0.0;
    rhs = 0.0;
    cost = 0.0;
  }
  double g;
  double rhs;
  double cost;

};

class state_hash {
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};

typedef std::priority_queue<state, std::vector<state>, std::greater<state> > ds_pq;
typedef __gnu_cxx::hash_map<state,cellInfo, state_hash, std::equal_to<state> > ds_ch;
typedef __gnu_cxx::hash_map<state, float, state_hash, std::equal_to<state> > ds_oh;


class DStarLite {
  
 public:

  enum MapType {MAP_RANDOM, MAP_GRADIENT};
    
  DStarLite();
  void   init(int sX, int sY, int gX, int gY);
  void   updateCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan();
  void   draw();
  void   drawCell(state s,float z);

  std::list<state> getPath(); // Returns path from start to goal.
  double getPathCost(double(*convertCellCost)(double)=NULL);

  bool getCost(int x, int y, double& cost);
  struct ipoint2 getLastSetGoal();
  void resetGoal();
  void createMap(int width, int height, double min_cost, double max_cost, 
      enum MapType map_type=MAP_RANDOM);
  
 private:
  
  std::list<state> path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;  

  ds_pq openList;
  ds_ch cellHash;
  ds_oh openHash;

  double mMapMinCost, mMapMaxCost;

  bool   close(double x, double y);
  void   makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void   setG(state u, double g);
  void setRHS(state u, double rhs);
  double eightCondist(state a, state b);
  int    computeShortestPath();
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b);
  state  calculateKey(state u);
  void   getSucc(state u, std::list<state> &s);
  void   getPred(state u, std::list<state> &s);
  double cost(state a, state b); 
  bool   occupied(state u);
  bool   isValid(state u);
  float  keyHashCode(state u);

  void createHill(int map_start_x, int map_stop_x, int map_start_y, int map_stop_y, 
      int hill_center_x, int hill_center_y, double max_cost, double cur_radius, double cur_cost);
  void smoothMap(int map_start_x, int map_stop_x, int map_start_y, int map_stop_y, int filter_size);

  inline double fRand(double min, double max)
  {
    double val = (double)rand() / RAND_MAX;
    return min + val * (max - min);
  }
};
} // namespace dstar_lite
#endif
