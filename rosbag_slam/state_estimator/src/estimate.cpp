#include "estimate.h"

Estimate::Estimate()
{

    sub_pose=nh.subscribe("/slam_out_pose", 1000, &Estimate::hector_slam_pose, this);
    sub_scan=nh.subscribe("/scan",1000,&Estimate::laser_scan_transform,this);
    sub_cloud=nh.subscribe("/my_cloud",10,&Estimate::transform_cloud,this);

    pub_scan = nh.advertise<sensor_msgs::LaserScan>("transformed_scan",10);

    disp=new display();
    stamp= ros::Time::now();
    num_state=6;
    num_mes=3;
    dt=0.1;
    lspc =new LaserScanToPointCloud(nh);

}

void Estimate::run(char *filename)
{
    /*1) hector slam provide only POSE estimation
     *2) KALMAN filter provides STATE estimation
     *3) Here we initialize the KALMAN filter parameter
     */
    ROS_INFO("Estmator thread started...");
    int n = num_state; // Number of states
    int m = num_mes; // Number of measurements
    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    A << 1,0,0,dt,0,0,
         0,1,0,0,dt,0,
         0,0,1,0,0,dt,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    C << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;

    Q << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;

    R<< 0,0,0,
        0,0,0,
        0,0,1;

    P=Q*0.01;

    Q=Q*0.001;

    kf =new KalmanFilter(dt,A, C, Q, R, P);
    kf->init();
    file=new file_reader(filename);



}

void Estimate::getMS()
{
//    implimented for debuging sensing rate
   ros::Time current = ros::Time::now();
   ROS_INFO_STREAM("time stamp in msec "<<(current.nsec - stamp.nsec)/1e6 );
   stamp=current;
}

void Estimate::laser_scan_transform(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /* 1) transform old rosbag data msg to new stamped msg
     * 2) Change the frameid in such a way that support tf transformation
     * 3) Display robot state in RVIZ
     * 4) Broadcast map to robot transformation using tf
     */

    Eigen::VectorXd X(num_state);
    X=kf->state().transpose();
    float state[3]={X[0],X[1],X[2]};


    tf::Transform transform;
    transform.setOrigin( tf::Vector3(state[0], state[1],1) );
    tf::Quaternion qq;
    qq.setRPY(0, 0,  state[2]);
    transform.setRotation(qq);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/robot_frame"));


    disp->robot_state(state);
    sensor_msgs::LaserScan newScan(*msg);
    newScan.header.frame_id="/robot_frame";
    newScan.header.stamp=ros::Time::now();
    pub_scan.publish(newScan);

}

void Estimate::hector_slam_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

/*1) GET YAW from quaternion using tf
 *2) update KALMAN FILTER with measurements
 */
    float px=msg->pose.position.x;
    float py=msg->pose.position.y;
    float ox = msg->pose.orientation.x;
    float oy = msg->pose.orientation.y;
    float oz = msg->pose.orientation.z;
    float ow = msg->pose.orientation.w;


    tf::Quaternion q(ox,oy,oz,ow);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double arr[]={px,py,yaw};
    Eigen::VectorXd y(num_mes);
    for(int i=0;i<3;i++)
        y[i]=arr[i];
    kf->update(y);

}

void Estimate::transform_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    Eigen::VectorXd mat(num_state);
    mat=kf->state().transpose();
    vector<float> vec(mat.data(), mat.data() + mat.size());
    for (int i=0;i<msg->data.size();i++)
        vec.push_back(msg->data[i]);
    file->write_log(vec);


}

