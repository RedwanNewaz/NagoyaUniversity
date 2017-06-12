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
#include "ros/ros.h"



class file_reader
{
public:
  file_reader();
  void read_file(char *filename);
  void getLine(int line, float *container);
  int totalLine();

private:
  int rows,cols;
  float **data;

protected:
  void print_database();

};

#endif // FILE_READER_H
