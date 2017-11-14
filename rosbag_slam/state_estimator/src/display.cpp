#include "display.h"

display::display()
{
    pub_robot = nh.advertise<visualization_msgs::Marker>("robot_trajectory", 10);

}

void display::robot_state(const float *state)
{


  robot.header.frame_id = line_list.header.frame_id = "/velodyne";
  robot.header.stamp = line_list.header.stamp = ros::Time::now();
  line_list.ns = "global_traj";
  robot.ns = "robot";
  robot.action = line_list.action =visualization_msgs::Marker::ADD;
  robot.id = 0;
  line_list.id = 1;
  robot.type = visualization_msgs::Marker::ARROW;
  line_list.type =visualization_msgs::Marker::LINE_STRIP;
  robot.scale.x = 1.0;
  robot.scale.y = 2.0;
  robot.scale.z = 0.1;
  line_list.scale.x = 0.5;
  line_list.scale.y = 0.5;
  line_list.scale.z = 0.5;
  // robot are green
   robot.color.b = 1.0f;
   robot.color.a = 1.0;
   line_list.color.r =0.5;
   line_list.color.b =1.0;
   line_list.color.a = 1.0;

   geometry_msgs::Point p;

   p.x=robot.pose.position.x =state[0];
   p.y=robot.pose.position.y =state[1];
   p.z=robot.pose.position.z = 1;

   tf::Quaternion q;
   q.setRPY(0,0,state[2]);
   robot.pose.orientation.x = q.x();
   robot.pose.orientation.y = q.y();
   robot.pose.orientation.z = q.z();
   robot.pose.orientation.w = q.w();
   robot.lifetime = ros::Duration();
   line_list.points.push_back(p);

   pub_robot.publish(robot);
   pub_robot.publish(line_list);


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
                std::swap(X[0],X[2]);
                std::swap(Y[0],Y[2]);
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

void display::get_samples(double **points, const float *robot){
   int n =100;
   ROS_INFO("populating data start");
   points=new double *[n];
   for (int i =0;i<n;i++){
       sample_lines[i].data = new float*[2];
       points[i]=new double[2];
//       ROS_INFO("robo {%.2f,%.2f} points {%.2lf,%.2lf}",robot[0],robot[1],points[i][0],points[i][1]);
//        ROS_INFO_STREAM(points[i][0]<<"\t"<<points[i][1]);

       float data[2][2]={{robot[0],points[i][0]},{robot[1],points[i][1]}};
       for (int j=0;j<2;j++){
           sample_lines[i].data[j] = new float[2];
            sample_lines[i].data[j][0]=data[j][0];
            sample_lines[i].data[j][1]=data[j][1];
       }
   }
   ROS_INFO("populating data finished");
}

void display::scan(double **points,const float *robot)
{
    //            Initialize local trajectory
    //------------------------------------------------------------------------------

     visualization_msgs::Marker local_trajectory;
     local_trajectory.header.frame_id = "/velodyne";
     local_trajectory.header.stamp = ros::Time::now();
     local_trajectory.ns ="trajectory";
     local_trajectory.type =visualization_msgs::Marker::LINE_LIST;
     local_trajectory.action =visualization_msgs::Marker::ADD;
     local_trajectory.scale.x = 0.1;
     local_trajectory.scale.y = 0.1;
     local_trajectory.scale.z = 0.1;
     local_trajectory.color.r =1.0;
     local_trajectory.color.a = 1.0;

      //        Decoding message
    //------------------------------------------------------------------------------
//    sample_init( robot[0],robot[1], robot[2]);
     get_samples(points, robot);
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
         pub_robot.publish(local_trajectory);
         delete[] sample_lines[i].data;
     }

//main_frame
     /*
     int n =100;
    points=new double *[n];
    for (int i =0;i<n;i++)
        points[i]=new double[2];
    ROS_INFO_STREAM("total point "<<n);
    geometry_msgs::Point robo;
    robo.x=robot[0];
    robo.y=robot[1];
    robo.z=1;
//    local_trajectory.points.clear();
    for (int i;i<n;++i){
         geometry_msgs::Point p;
        p.x=points[i][0];
        p.y=points[i][1];
        p.z=1;
        local_trajectory.points.push_back(robo);
        local_trajectory.points.push_back(p);
        local_trajectory.id = 1;

    }
    pub_robot.publish(local_trajectory);

*/
}
