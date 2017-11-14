#include "file_reader.h"
using namespace std;
file_reader::file_reader(char *file)
{
filename=file;
ROS_INFO_STREAM("file:= "<<file_exists(file));
}

void file_reader::write_log(vector<float> &data)
{
    ofstream myfile;
    myfile.open (filename, std::ofstream::out | std::ofstream::app);
    for(int j =0; j<data.size(); j++){
        myfile<<data[j];
        if(j!=data.size())
        myfile<<",";}
    myfile<<"\n";
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

void file_reader::save_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){

  ofstream myfile;
  char *filename="/home/redwan/catkin_ws/src/traj_saver/launch/map.txt";
  myfile.open (filename);
  ROS_INFO("MAP RECIVED! Saving @ \n %s ",filename);
  ROS_INFO("Origin {%f,%f}",msg->info.origin.position.x,msg->info.origin.position.y);

  int width,height;
  width = msg->info.width;
  height = msg->info.height;
  float MAP[height][width];

  for(int i = 0; i<height; i++)
  {
      for(int j =0; j<width; j++){
          MAP[i][j]=msg->data[i*width+j];
          myfile<<MAP[i][j];
          if(j!=width)
          myfile<<",";
      }
      myfile<<"\n";
  }
  myfile.close();

}
