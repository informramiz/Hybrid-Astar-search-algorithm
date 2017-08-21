/*
 * utils.h
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <vector>
using namespace std;

class Utils {
public:
  Utils();
  virtual ~Utils();

  /**
   * Method to print a 2D vector. Vector type can be either primitive or
   * a custom type that has overloaded stream insertion operator (<<).
   */
  template<class T>
  static void print_grid(const vector<vector<T> > &grid) {
    for(int i = 0; i < grid.size(); i++)
    {
      cout << grid[i][0];
      for(int j = 1; j < grid[0].size(); j++)
      {
        cout << "," << grid[i][j];
      }
      cout << endl;
    }
  }

  /**
   * Method to print a 1D vector. Vector type can be either primitive or
   * a custom type that has overloaded stream insertion operator (<<).
   */
  template<class T>
  static void print_vector(const vector<T> &values) {
    for (int i = 0; i < values.size(); ++i) {
      cout << values[i];

      if (i != values.size() - 1) {
        cout << ",";
      }
    }

    cout << endl;
  }

  /**
   * Calculates Euclidean distance
   */
  static double euclidean(int x1, int y1, int x2, int y2);

  /**
   * Calculates Euclidean distance in 3D
   */
  static double euclidean_3d(int x1, int y1, double theta_rad1, int x2, int y2, double theta_rad2);
};

#endif /* UTILS_H_ */
