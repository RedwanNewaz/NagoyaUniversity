#include "display.h"
using namespace std;
display::display()
{
  line_list =new visualization_msgs::Marker;
  robot =new visualization_msgs::Marker;

  marker_pub = nh.advertise<visualization_msgs::Marker>("robot_trajectory", 10);
  line_list->header.frame_id=robot->header.frame_id= "/velodyne";

  line_list->ns ="trajectory"; line_list->id = 1;
  robot->ns = "robot"; robot->id = 0;
  robot->type = visualization_msgs::Marker::ARROW;
  line_list->type =visualization_msgs::Marker::LINE_STRIP;
  robot->action =line_list->action =visualization_msgs::Marker::ADD;
  robot->scale.x = 1.0;      line_list->scale.x = 0.5;
  robot->scale.y = 2.0;      line_list->scale.y = 0.5;
  robot->scale.z = 0.1;      line_list->scale.z = 0.5;

  robot->color.b = 1.0f;     line_list->color.r =0.5;
  robot->color.a = 1.0;      line_list->color.b =1.0;
                             line_list ->color.a = 1.0;

// wait for rviz to open
  ROS_INFO("waiting for rviz.......");
  while (marker_pub.getNumSubscribers() <= 0 );

}






void display::publishRobot( const geometry_msgs::Pose msg){

     line_list->header.stamp=robot->header.stamp = ros::Time::now();
     geometry_msgs::Point p;
     robot->pose.position.x=p.x= msg.position.x;
     robot->pose.position.y=p.y= msg.position.y;
     robot->pose.orientation.x=msg.orientation.x;
     robot->pose.orientation.y=msg.orientation.y;
     robot->pose.orientation.z=msg.orientation.z;
     robot->pose.orientation.w=msg.orientation.w;
     robot->pose.position.z=p.z=1;
     robot->lifetime = ros::Duration();
     line_list->points.push_back(p);
     marker_pub.publish(*robot);
     marker_pub.publish(*line_list);

}


void display::sample_init(float x,float y, float yaw)
{
//                            points along the heading
//------------------------------------------------------------------------------

    float cord[SAMPLE_LINES][2];
    int count=0;
    float angle[SAMPLE_LINES];
    for(float theta=-M_PI/2+yaw;theta<M_PI/2+yaw;theta+=(M_PI/SAMPLE_LINES)){
      if(count>SAMPLE_LINES)break;
        cord[count][0]= x+R*cos(theta);
        cord[count][1]= y+R*sin(theta);
        angle[count]=theta;
        count++;
    }
//                            sorting candidate points
//------------------------------------------------------------------------------
    std::vector<double> X(3), Y(3);
    for(int i=0;i<SAMPLE_LINES;i++){
        float xm=x+R*cos(angle[i])/2;
        float ym=y+R*sin(angle[i])/2;//+0.75-sin(angle[i]);//modify 0.75-sin() part for spline angle
        X[0]=x;Y[0]=y;
        X[1]=xm;Y[1]=ym;
        X[2]=cord[i][0];Y[2]=cord[i][1];
            if(X[0]>X[2]){
                swap(X[0],X[2]);
                swap(Y[0],Y[2]);
            }
            if(X[0]>=X[2]||X[1]<=X[0]||X[1]>=X[2]){
                   ROS_INFO_STREAM("invalid points : "<<X[0]<<"\t"<<X[1]<<"\t"<<X[2]);
                continue;
            }
//                            interpolated spline points
//------------------------------------------------------------------------------
        tk::spline s;
        s.set_points(X,Y);
        float sizeX=std::max(x,cord[i][0])-std::min(x,cord[i][0]);
        sample_lines[i].data = new float*[SPLINE_POINTS];
        sample_lines[i].row=0;
        sample_lines[i].col=2;
        for (float j=std::min(x,cord[i][0]);j<std::max(x,cord[i][0]);j+=sizeX/SPLINE_POINTS){
            int n=sample_lines[i].row;
            sample_lines[i].data[n] = new float[2];
            sample_lines[i].data[n][0]=j;
            sample_lines[i].data[n][1]=s(j);
            sample_lines[i].row++;
            if(sample_lines[i].row>=SPLINE_POINTS)break;
        }
    }


}




void display::get_rotated_points(float x, float y, float *x1, float *y1, float theta, int n)
{

  for(int i=0;i<n;i++){
    float x_changed= x1[i]*cos(theta);
    float y_changed= y1[i]*sin(theta);
    x1[i]=x+x_changed;
    y1[i]=y+y_changed;
  }

}

void display::sample_points(const geometry_msgs::Pose msg)
{
//            Initialize local trajectory
//------------------------------------------------------------------------------

 visualization_msgs::Marker local_trajectory;
 local_trajectory.header.frame_id = "/velodyne";
 local_trajectory.header.stamp = ros::Time::now();
 local_trajectory.ns ="local_traj";
 local_trajectory.type =visualization_msgs::Marker::LINE_LIST;
 local_trajectory.action =visualization_msgs::Marker::ADD;
 local_trajectory.scale.x = 0.1;
 local_trajectory.scale.y = 0.1;
 local_trajectory.scale.z = 0.1;
 local_trajectory.color.g =1.0;
 local_trajectory.color.a = 1.0;

  //        Decoding message
//------------------------------------------------------------------------------

  float x = msg.position.x;
  float y = msg.position.y;
  tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  sample_init( x,y, yaw);
  //      Populating trajectory
//------------------------------------------------------------------------------
  for(int i=0;i<SAMPLE_LINES;i++){
      geometry_msgs::Point points;
      int sn=sample_lines[i].row;
      for(int j=0;j<sn;j++){
        points.x=sample_lines[i].data[j][0];points.y=sample_lines[i].data[j][1];points.z=1;
        local_trajectory.points.push_back(points);
      }

      //      Publishing trajectory
//------------------------------------------------------------------------------

      if(local_trajectory.points.size()%2!=0)
          local_trajectory.points.pop_back();
      local_trajectory.id = i;
      marker_pub.publish(local_trajectory);
      delete[] sample_lines[i].data;
  }


}



