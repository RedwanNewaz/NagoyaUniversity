#ifndef FILE_READER_H
#define FILE_READER_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include <sys/stat.h>
#include <unistd.h>
#include <string>

inline bool file_exists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}
class file_reader
{
public:
  file_reader(char *file);
  void write_log(std::vector<float>& data);
  void read_file(char *filename);
  void getLine(int line, float *container);
  void save_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  int totalLine();

private:
  int rows,cols;
  float **data;
  char *filename;

protected:
  void print_database();

};

#endif // FILE_READER_H
