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

    vector< vector< vector<maze_s> > > closed;
    vector< vector< vector<maze_s> > > came_from;
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

  maze_path search(vector< vector<int> > grid, vector<double> start, vector<int> goal);

  vector<maze_s> reconstruct_path(vector< vector< vector<maze_s> > > came_from, vector<double> start, HBF::maze_s final);
  void print_grid(const vector<vector<double> > &grid);
private:
  bool is_valid_cell(double x2, double y2,
                     const vector<vector<int> >& grid);
  double deg2rad(double delta_i);

  vector<vector <double> > calculate_euclidean_heuristic(const vector<vector<int> > &grid, const vector<int> &goal);
  double euclidean(int x1, int y1, int x2, int y2);

  /**
   * ---- Non-holonomic without obstacles heuristic-----
   *
   * Calculates 3D (x, y, theta) grid of heuristic using euclidean distance. This heuristic can be used to achieve
   * goal in desired heading (theta)
   */
  vector<vector<vector<double> > > calculate_euclidean_heuristic_3d(const vector<vector<int> > &grid, const vector<int> &goal);
  double euclidean_3d(int x1, int y1, double theta_rad1, int x2, int y2, double theta_rad2);
};

#endif /* HYBRID_BREADTH_FIRST_H_ */
