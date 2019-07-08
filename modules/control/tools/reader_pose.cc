#include <fstream>
#include <stdlib.h>
#include <malloc.h>
#include <stdio.h>
#include <memory>
#include <string>
#include <cstring>
#include <iostream>
#include <vector>
#include <iterator>


struct Point{
  double x,z,y;
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
  //buffer = (char*)malloc(size_v);
  std::cout << "length is " << size_v << std::endl;
  input_file.read(buffer, size_v);
  //std::cout << "read result:" << result << std::endl;
  input_file.close();
  for (int i = 0; i < size_v; i += 24){
        double test_double = 0;
        memcpy(&test_double, buffer, 8);
	std::memcpy(&temp_vec, buffer + i, 24);
	new_vector.push_back(temp_vec);
  }

  for(const auto& v:new_vector) {
    std::cout << v.x << ", " << v.z << " , "<< v.y << std::endl;
  }
  return 0;
}
