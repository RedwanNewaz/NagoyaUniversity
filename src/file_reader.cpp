#include "file_reader.h"
using namespace std;
file_reader::file_reader()
{

}

void file_reader::read_file(char *filename){
  float **database;
  database = new float* [200000];
  ifstream myfile(filename);
  string line;int clm,row=0;
  while ( getline (myfile,line) ){
    database[row] = new float[10];
    string temp;int col=0; char* sz;istringstream f(line);
    while(getline(f, temp, ',')){
       // store data
       database[row][col]=strtof(temp.c_str(),&sz);
      col++;
      clm=col;
      }

    row++;
  }

//  populating to long term data
  rows = row; cols =clm;
  ROS_INFO("filesize:=[%dx%d]",rows,cols);
  data = new float* [row];
  for (int x=0; x<rows; x++){
    data[x] = new float[cols];
    for (int y=0; y<clm; y++)
        data[x][y]=database[x][y];

}
  delete[] database;
//    print_database();
}

void file_reader::getLine(int line, float *container)
{
  for (int y=0; y<cols; y++)
    container[y]=data[line][y];
}

int file_reader::totalLine()
{
  return rows;
}

void file_reader::print_database()
{
      for (int x=0; x<rows; x++){
        for (int y=0; y<cols; y++)
            cout<<data[x][y] <<" ";
      cout<<endl;
    }
}
