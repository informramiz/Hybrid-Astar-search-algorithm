/*
 * utils.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "utils.h"

Utils::Utils() {
  // TODO Auto-generated constructor stub

}

Utils::~Utils() {
  // TODO Auto-generated destructor stub
}

double Utils::euclidean(int x1, int y1, int x2, int y2) {
  //  int dist_x = abs(x1 - x2);
  //  int dist_y =  abs(y1 - y2);
  //  int dist = dist_x + dist_y;
  //
  //  return dist;

  int dist_x = (x1 - x2);
  int dist_y =  (y1 - y2);
  int squared_dist = pow(dist_x, 2) + pow(dist_y, 2);

  return sqrt(squared_dist);
}

double Utils::euclidean_3d(int x1, int y1, double theta_rad1, int x2, int y2, double theta_rad2) {
  int dist_x = (x1 - x2);
  int dist_y =  (y1 - y2);
  double dist_theta = (theta_rad1 - theta_rad2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_theta, 2);

  return sqrt(squared_dist);
}



