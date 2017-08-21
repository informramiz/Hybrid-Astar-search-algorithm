/*
 * hybrid_breadth_first.h
 *
 *  Created on: Aug 19, 2017
 *      Author: ramiz
 */

#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>

using namespace std;

class HBF {
public:

  int NUM_THETA_CELLS = 90;
  double SPEED = 1.45;
  double LENGTH = 0.5;

  struct maze_s {

    int g;  // iteration
    double x;
    double y;
    double theta;
  };

  struct maze_path {

    vector<vector<vector<maze_s> > > closed;
    vector<vector<vector<maze_s> > > came_from;
    maze_s final;

  };

  /**
   * Constructor
   */
  HBF();

  /**
   * Destructor
   */
  virtual ~HBF();

  int theta_to_stack_number(double theta);

  int idx(double float_num);

  vector<maze_s> expand(maze_s state);

  maze_path search(vector<vector<int> > grid, vector<double> start,
                   vector<int> goal);

  vector<maze_s> reconstruct_path(vector<vector<vector<maze_s> > > came_from,
                                  vector<double> start, HBF::maze_s final);

  vector<vector<double> > calculate_euclidean_heuristic(
      const vector<vector<int> > &grid, const vector<int> &goal);

  /**
   * ---- Non-holonomic without obstacles heuristic-----
   *
   * Calculates 3D (x, y, theta) grid of heuristic using euclidean distance. This heuristic can be used to achieve
   * goal in desired heading (theta)
   */
  vector<vector<vector<double> > > calculate_euclidean_heuristic_3d(
      const vector<vector<int> > &grid, const vector<int> &goal);

  /**
   * ---Holonomic with obstacles heuristic, calculated using dynamic programming---
   */
  vector<vector<int> > holonomic_min_path_cost_from_each_cell_heuristic(
      const vector<vector<int> > &grid, const vector<int> &goal);
  /**
   * Returns cost for shortest path from given cell to goal.
   * This is just a wrapper on top of f_shortest_path_cost() method
   */
  vector<vector<int> > holonomic_min_cost_from_cell(
      const vector<vector<int> > &grid, const vector<int> &start,
      const vector<int> &goal);

private:
  bool is_valid_cell(double x2, double y2, const vector<vector<int> >& grid);
  bool is_valid_cell(const vector<vector<int> >& grid, const vector<int> &cell);

  /**
   * Returns cost for shortest path from given cell to goal.
   * @out cost_grid will contain the cost for each cell visited.
   */
  int holonomic_f_shortest_path_cost(vector<vector<int> > &grid,
                                     vector<vector<int> > &cost_grid,
                                     const vector<int> &cell,
                                     const vector<int> &goal);

  /**
   * Returns the next cell after making given move from current cell
   */
  vector<int> make_move(const vector<int> &current_cell, const vector<int> &move);

  vector<vector<int> > holomonic_moves_ = { { -1, 0 }, //go up
      { 1, 0 }, //go down
      { 0, 1 }, //go left
      { 0, -1 }, //go right
      };

  vector<string> holomonic_moves_names_ = { "Up", "Down", "Left", "Right" };

  //define indexes for above moves for easy access
  const int MOVE_UP_INDEX = 0;
  const int MOVE_DOWN_INDEX = 1;
  const int MOVE_LEFT_INDEX = 2;
  const int MOVE_RIGHT_INDEX = 3;

  /**
   * Cost for an obstacle
   */
  const int COST_FOR_OBSTACLE = 999999;
};

#endif /* HYBRID_BREADTH_FIRST_H_ */
