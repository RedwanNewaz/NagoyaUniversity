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

void display::spline_trajectory(const geometry_msgs::Pose msg){

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


  //-----------------------------------------------

  float x = msg.position.x;
  float y = msg.position.y;
  const int n=15;
  const float R =5;
//ROS_INFO("local trajectories publishing");
  //-----------------------------------------------
//  orientation fixed
  tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);



//  spline operation
  //----------------------------------------------------------------------------
  float cord[n][2];
//% get circular points
  int count=0;
  float angle[n];
  float thetas=3*M_PI/2+q.getAngle();
  ROS_WARN("Theta: %f",q.getAngle());
  for(float theta=-1*M_PI/2+q.getAngle();theta<1*M_PI/2+q.getAngle();theta+=0.2){
    if(count>n)continue;
      cord[count][0]= x+R*cos(theta);
      cord[count][1]= y+R*sin(theta);
      angle[count]=theta;
      count++;
  }
//  % GET SPLINE LINES
  std::vector<double> X(3), Y(3);
  for(int i=0;i<count;i++){
//      % get middle point
//      float xm=(x+cord[i][0])/2;
      float xm=x+R*cos(angle[i])/2;
      float ym=y+R*sin(angle[i])/2+0.75-sin(angle[i]);
//      swap(xm,ym);
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

       //      get a new marker
        local_trajectory.id = i;
// get smooth trajectory
          //populate value with a container
            geometry_msgs::Point points;
      tk::spline s;
      s.set_points(X,Y);
      for (float j=std::min(x,cord[i][0]);j<std::max(x,cord[i][0]);j+=0.001){
//           printf("spline at %f is %f\n", j, s(j));
           points.x=j;points.y=s(j);points.z=1;
           local_trajectory.points.push_back(points);
      }
//      remove odd number
      if(local_trajectory.points.size()%2!=0){
        local_trajectory.points.pop_back();
      }
marker_pub.publish(local_trajectory);

}

  //----------------------------------------------------------------------------
}
