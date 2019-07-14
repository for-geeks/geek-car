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
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

struct Point {
  double x, z, y;
};

int main(int argc, char* argv[]) {
  std::vector<Point> new_vector;

  std::string file_name = "/home/raosiyue/pose.dat";

  std::ifstream input_file(file_name, std::ios::in | std::ios::binary);
  input_file.seekg(0, std::ios_base::end);
  auto size_v = input_file.tellg();
  input_file.seekg(0, std::ios_base::beg);
  Point temp_vec;
  char buffer[100000];
  // buffer = (char*)malloc(size_v);
  std::cout << "length is " << size_v << std::endl;
  input_file.read(buffer, size_v);
  // std::cout << "read result:" << result << std::endl;
  input_file.close();
  for (int i = 0; i < size_v; i += 24) {
    double test_double = 0;
    std::memcpy(&test_double, buffer, 8);
    std::memcpy(&temp_vec, buffer + i, 24);
    new_vector.push_back(temp_vec);
  }

  for (const auto& v : new_vector) {
    std::cout << v.x << ", " << v.z << " , " << v.y << std::endl;
  }
  return 0;
}
