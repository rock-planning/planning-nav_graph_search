/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 * v1.0 modified by Stefan Haase (stefan.haase@dfki.de)
 */

#include "DStarLite.h"
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <set>

#ifdef USE_OPEN_GL
#ifdef MACOS
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace std;
using namespace __gnu_cxx;
using namespace dstar_lite;

/* void DStarLite::DStarLite()
 * --------------------------
 * Constructor sets constants.
 */
DStarLite::DStarLite() { 
  maxSteps = 800000;  // node expansions before we give up
  C1       = 1;      // cost of an unseen cell
  mMapMinCost = 0;
  mMapMaxCost = 0;
}

/* float DStarLite::keyHashCode(state u) 
 * -------------------------- 
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float DStarLite::keyHashCode(state u) {
  float ret = (float)(u.k.first + 1193*u.k.second);
  return ret;

}

/* bool DStarLite::isValid(state u) 
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool DStarLite::isValid(state u) {
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;
  
}

/* void DStarLite::getPath() 
 * --------------------------
 * Returns the path created by replan()
 */
list<state> DStarLite::getPath() {
  return path;
}

/* void DStarLite::getPath() 
 * --------------------------
 * \param convertCellCost Function can be passed to convert each internal cell-cost-value.
 * \return The current path costs.
 */
double DStarLite::getPathCost(double(*convertCellCost)(double)) {
  // Calculate costs (time) for current path.
  double path_cost = 0;
  std::list<dstar_lite::state>::iterator it_first = path.begin(), it_second = path.begin();
  it_second++;
  double cost_tmp = 0;
  for(; it_second != path.end(); it_first++, it_second++) {
      cost_tmp = cost(*it_first, *it_second);
      if(convertCellCost != NULL) {
        cost_tmp = convertCellCost(cost_tmp);
      }
      path_cost += cost_tmp;
  }
  return path_cost;
}

/* bool DStarLite::getCost(int x, int y, double& cost)
 * --------------------------
 * Requests the cost of the cell. 
 */
bool DStarLite::getCost(int x, int y, double& cost) {
  state u;
  u.x = x;
  u.y = y;

  ds_ch::iterator it = cellHash.find(u);
  if(it == cellHash.end()) {
    return false;
  } 
  cost = it->second.cost;
  return true;
}

/* struct ipoint2 getLastSetGoal() 
 * --------------------------
 * Returns the current set goal.
 */
struct ipoint2 DStarLite::getLastSetGoal() {
  struct ipoint2 goal_tmp;
  goal_tmp.x = s_goal.x;
  goal_tmp.y = s_goal.y;
  return goal_tmp;
}

/* void resetGoal()
 * --------------------------
 * Resets the goal using the last set position.
 */
void DStarLite::resetGoal() {
  updateGoal(s_goal.x, s_goal.y);
}

void DStarLite::createMap(int width, int height, double min_cost, double max_cost, 
      enum MapType map_type) {

  if(min_cost > max_cost) {
    double temp = min_cost;
    min_cost = max_cost;
    max_cost = temp;
  }

  // Used for drawing.
  mMapMinCost = min_cost;
  mMapMaxCost = max_cost;

  double shift = 0;
  if(min_cost < 0) {
    shift = fabs(min_cost);
    mMapMinCost = min_cost = 0;
    max_cost += shift;      
  }

  srand (time(NULL));
  double cell_cost;
  int x_start = s_start.x - width/2;
  int x_end = s_start.x + width/2;
  int y_start = s_start.y - height/2;
  int y_end = s_start.y + height/2;

  // Initialize map with C1 and drawing a border.
  for(int x = x_start; x < x_end; ++x) {
    for(int y = y_start; y < y_end; ++y) {
      cell_cost = C1;
      if(x == x_start || x == x_end-1 || y == y_start || y == y_end-1) {
        cell_cost = -1;
      }
      updateCell(x,y,cell_cost);
    }
  }  

  switch(map_type) {
    case MAP_GRADIENT: {
      int hill_center_x = 0;
      int hill_center_y = 0;
      int hill_radius;
      for(int i=0; i<width; ++i) {
        hill_radius = (rand() % 10) + 1;
        hill_center_x = rand()%(x_end-x_start) + x_start;
        hill_center_y = rand()%(y_end - y_start) + y_start;
        createHill(x_start+1, x_end-1, y_start+1, y_end-1,
            hill_center_x, hill_center_y, max_cost, hill_radius, C1+1);
      }
      break;
    }
    case MAP_RANDOM: {
      for(int x=x_start+1; x<x_end-1; ++x) {
        for(int y=y_start+1; y<y_end-1; ++y) {
          cell_cost = fRand(min_cost, max_cost) - shift;
          // Create border arounf the map.
          if(x == x_start || x == x_end-1 || y == y_start || y == y_end-1) {
            cell_cost = -1;
          }
          updateCell(x,y,cell_cost < 1 ? -1 : cell_cost);
        }
      }
      smoothMap(x_start+1, x_end-1, y_start+1, y_end-1, 2);
      break;
    }
  }
}

void DStarLite::createHill(int map_start_x, int map_stop_x, int map_start_y, int map_stop_y, 
    int hill_center_x, int hill_center_y, double max_cost, double cur_radius, double cur_cost) {
  if(cur_radius <= 0)
    return;

  if(cur_cost > max_cost)
    cur_cost = max_cost;

  int start_x = hill_center_x - cur_radius;
  int stop_x = hill_center_x + cur_radius;
  int start_y = hill_center_y - cur_radius;
  int stop_y = hill_center_y + cur_radius;

  if(start_x < map_start_x) start_x = map_start_x;
  if(start_y < map_start_y) start_y = map_start_y;
  if(stop_x > map_stop_x) stop_x = map_stop_x; // -1 ?
  if(stop_y > map_stop_y) stop_y = map_stop_y; // -1 ?

  double dist = 0;
  double radius_2 = cur_radius * cur_radius;
  for(int x = start_x; x < stop_x; ++x) {
    for(int y = start_y; y < stop_y; ++y) {
      dist = (hill_center_x - x) * (hill_center_x - x) + 
          (hill_center_y - y) * (hill_center_y - y);
      if(dist <= radius_2) {
        updateCell(x,y,cur_cost < 1 ? -1 : cur_cost);
      }
    }
  }  
  createHill(map_start_x, map_stop_x, map_start_y, map_stop_y, hill_center_x, hill_center_y, max_cost, cur_radius-1, cur_cost+1);
}

void DStarLite::smoothMap(int map_start_x, int map_stop_x, int map_start_y, int map_stop_y, int filter_size) {
  double sum = 0;
  double cost = 0;
  double affected_cell_counter = 0;
  for(int x = map_start_x + filter_size; x < map_stop_x - filter_size; ++x) {
    for(int y = map_start_y + filter_size; y < map_stop_y - filter_size; ++y) {
      sum = 0;
      affected_cell_counter = 0;
      getCost(x, y, cost);
      if(cost < 1) // Ignore obstacles. 
        continue;
      for(int x_f = x - filter_size; x_f < x + filter_size; ++x_f) {
        for(int y_f = y - filter_size; y_f < y + filter_size; ++y_f) {
          getCost(x_f, y_f, cost);   
          if(cost > 1) { // Only use free cells for filtering.
            sum += cost;
            affected_cell_counter++;
          }
        }
      }
      updateCell(x,y, sum / affected_cell_counter);
    }
  }
}


////////////// PRIVATE //////////////

/* bool DStarLite::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool DStarLite::occupied(state u) {
  ds_ch::iterator cur = cellHash.find(u);
  
  if (cur == cellHash.end()) {
    return false;
  }
  bool ret = cur->second.cost < 0;
  return ret;
}

/* void DStarLite::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void DStarLite::init(int sX, int sY, int gX, int gY) {
  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();

  k_m = 0;
  
  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;
}
/* void DStarLite::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void DStarLite::makeNewCell(state u) {
  
  if (cellHash.find(u) != cellHash.end()) {
    return;
  }
  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;
}

/* double DStarLite::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double DStarLite::getG(state u) {
  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].g;
  
}

/* double DStarLite::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double DStarLite::getRHS(state u) {
  if (u == s_goal) return 0;  

  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].rhs;
  
}

/* void DStarLite::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void DStarLite::setG(state u, double g) {
  makeNewCell(u);  
  cellHash[u].g = g; 
}

/* void DStarLite::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
void DStarLite::setRHS(state u, double rhs) {
  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double DStarLite::eightCondist(state a, state b) 
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 * Regards cell movement, e.g. the cost for (0,0) to (3,2) would be
 * 3.82 (two diagonal and one horizontal movement) instead of the
 * euklidian distance 3.6.
 */
double DStarLite::eightCondist(state a, state b) {
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* int DStarLite::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles. 
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int DStarLite::computeShortestPath() {
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) {
    return 1;
  }

  int k=0;
 
  while (!openList.empty() && 
         (openList.top() < (s_start = calculateKey(s_start)) || 
          getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps, no path found\n");
      return -1;
    }

    state u;

    bool test = (getRHS(s_start) != getG(s_start));
    
    // lazy remove
    while(1) { 
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();
      
      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }
  
    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool DStarLite::close(double x, double y) 
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool DStarLite::close(double x, double y) {
  if (isinf(x) && isinf(y)) {
    return true;
  }
  bool ret = (fabs(x-y) < 0.00001);
  return ret;
  
}

/* void DStarLite::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void DStarLite::updateVertex(state u) {
  list<state> s;
  list<state>::iterator i;
 
  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;
    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) insert(u);
  
}

/* void DStarLite::insert(state u) 
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void DStarLite::insert(state u) {
  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;
  
  openHash[u] = csum;
  openList.push(u);
} 

/* void DStarLite::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void DStarLite::remove(state u) {
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double DStarLite::trueDist(state a, state b) 
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double DStarLite::trueDist(state a, state b) {
  
  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);
  
}

/* double DStarLite::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double DStarLite::heuristic(state a, state b) {
  //return eightCondist(a,b);
  //return trueDist(a,b)*C1;
  return eightCondist(a,b)*C1;
}

/* state DStarLite::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state DStarLite::calculateKey(state u) {
  double val = fmin(getRHS(u),getG(u));
  
  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double DStarLite::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double DStarLite::cost(state a, state b) {
  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1)
    scale = M_SQRT2;

  if (cellHash.count(a) == 0)
    return scale*C1;

  return scale*cellHash[a].cost;

}
/* void DStarLite::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 *
 * Warning: Set to [0,1) causes problems, resulting in looping pathes.
 */
void DStarLite::updateCell(int x, int y, double val) {
  state u;
  
  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  if(val >= 0 && val < 1.0) {
    fprintf(stderr, "Cell value has to be < 0 or >= 1\n");
    return;
  }

  makeNewCell(u); 
  cellHash[u].cost = val;

  updateVertex(u);
}

/* void DStarLite::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors. 
 */
void DStarLite::getSucc(state u,list<state> &s) {
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

}

/* void DStarLite::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void DStarLite::getPred(state u,list<state> &s) {
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  
}

/* void DStarLite::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void DStarLite::updateStart(int x, int y) {
  double cost = 0.0;
  if(!getCost(x,y,cost)) { // Cell has not been added yet.
    updateCell(x, y, C1);
  }

  if(cost < 0) {
    fprintf(stderr, "Start has been set on an obstacle, obstacle will be removed\n");
    updateCell(x, y, C1);
  }

  s_start.x = x;
  s_start.y = y;

  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;
  
}

/* void DStarLite::updateGoal(int x,
 int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void DStarLite::updateGoal(int x, int y) {
  //LOG_DEBUG("DStar-Lite updateGoal");
  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;
  
  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;
  

  double cost = 0.0;

  // Request old start and goal cost values.
  double start_cost = cellHash[s_start].cost;
  state next_goal_tmp(x,y);
  double next_goal_cost = C1;
  ds_ch::iterator it = cellHash.find(next_goal_tmp);
  if(it != cellHash.end()) {
    next_goal_cost = it->second.cost;
  } 

  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      cost = i->second.cost;

      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();
  
  k_m = 0;

  // Update goal.
  s_goal.x = x;
  s_goal.y = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = next_goal_cost;
  cellHash[s_goal] = tmp;

  // Update start.
  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = start_cost;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;    

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }
  

}

/* bool DStarLite::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 * the metric euclidean(state, goal) + euclidean(state,start). 
 *
 * If false is returned in the current version, the goal should be reset 
 * and a replan should be initiated at least once.
 */
bool DStarLite::replan() {
  path.clear();

  int res = computeShortestPath();

  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL: res < 0\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start; 

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL: isinf(getG(s_start)\n");
    return false;
  }

  set<ipoint2> set_path;
  pair<set<ipoint2>::iterator, bool> set_ret;
  
  while(cur != s_goal) {
    
    path.push_back(cur);
    //fprintf(stderr, "added (%d, %d, %4.2f, %4.2f)\n",cur.x, cur.y, cur.k.first, cur.k.second);

    ipoint2 point(cur.x, cur.y);
    set_ret = set_path.insert(point);
    if(!set_ret.second) {
      fprintf(stderr, "Loop detected during path creation (%d,%d)\n", set_ret.first->x, set_ret.first->y);
      set<ipoint2>::iterator it = set_path.begin();
      for(; it != set_path.end(); it++) {
        fprintf(stderr, "(%d,%d) ", it->x, it->y);
      }
      fprintf(stderr, "(%d,%d) ", s_goal.x, s_goal.y);
      fprintf(stderr, "\n");   
      return false;
    }

    getSucc(cur, n); // Returns all 8 successors. 

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL: n.empty()\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin = 0.0;
    state smin;
    
    for (i=n.begin(); i!=n.end(); i++) {
  
      //if (occupied(*i)) continue;
      double val  = cost(cur,*i); // cost going from 'cur' to neighbour 'i', cost of going from cur.
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);
      
      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}

#ifdef USE_OPEN_GL

void DStarLite::draw() {
  ds_ch::iterator iter;
  ds_oh::iterator iter1;
  state t;

  list<state>::iterator iter2;
  
  glBegin(GL_QUADS);
  for(iter=cellHash.begin(); iter != cellHash.end(); iter++) {
    if (iter->second.cost == 1) {
      glColor3f(0,1,0);
    }
    else if (iter->second.cost < 0 ) {
      glColor3f(1,0,0);
    }
    else if (iter->second.cost == 0 ) {
      glColor3f(0.6,0.6,0.6);
    }
    else {
      double color_b = mMapMaxCost == 0 ? 1.0 : iter->second.cost / mMapMaxCost;
      glColor3f(0,0,color_b);
    }
    drawCell(iter->first,0.45);
  }

  glColor3f(1,1,0);
  drawCell(s_start,0.45);
  glColor3f(1,0,1);
  drawCell(s_goal,0.45);

  for(iter1=openHash.begin(); iter1 != openHash.end(); iter1++) {
    glColor3f(0.4,0,0.8);
    drawCell(iter1->first, 0.2);
  }

  
  glEnd();


  glLineWidth(4);
  glBegin(GL_LINE_STRIP);
  glColor3f(0.6, 0.1, 0.4);

  for(iter2=path.begin(); iter2 != path.end(); iter2++) {
    glVertex3f(iter2->x, iter2->y, 0.2);
  }
  glEnd();

}

void DStarLite::drawCell(state s, float size) {
  float x = s.x;
  float y = s.y;
  
  
  glVertex2f(x - size, y - size);
  glVertex2f(x + size, y - size);
  glVertex2f(x + size, y + size);
  glVertex2f(x - size, y + size);


}

#else
void DStarLite::draw() {
  fprintf(stderr, "Calling empty DStar-Lite draw function");
}
void DStarLite::drawCell(state s, float z) {
  fprintf(stderr, "Calling empty DStar-Lite drawCell function");
}
#endif
