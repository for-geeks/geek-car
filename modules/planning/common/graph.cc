/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/
#include <iostream>

namespace apollo {
namespace planning {
namespace common {

const double LaneWidth = 0.4;  // meters
const double NextStep = 0.1;   // meters

struct Point {
  int x, y;
};

struct Graph {
  std::vector<double> left;
  std::vector<double> right;

  Graph(std::vector<double> left, std::vector<double> right)
      : left(left), right(right) {}

  bool in_lane(Point p){return 0 <= p.x < LaneWidth}

  std::vector<Point> neighbors(Point p) {
    std::vector<Point> result;
    Point next{p.x + NextStep, p.y};
    if (in_lane(next)) {
      return result;
    }
  }

  int cost(Point p) { return p.x ? 5 : 1; }
};

}  // namespace common
}  // namespace planning
}  // namespace apollo
