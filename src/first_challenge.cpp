#include "first_challenge/first_challenge.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz_", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::run()
{

    if(old_position.x - odometry_pose.pose.position.x > 1.0){
        cmd_vel_.mode = 11;
        cmd_vel_.cntl.linear.x = 0.1;
        cmd_vel_.cntl.angular.z = 0.7;

        pub_cmd_vel_.publish(cmd_vel_);
    }
}

void FirstChallenge::show_odom()
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan()
{
    float range_min = 1e6;
    for (int i = 0; i < laser_.ranges.size(); i++) {
        if (laser_.ranges[i] < range_min) {
            range_min = laser_.ranges[i];
        }
    }
    // ROS_INFO_STREAM("scan: min: %f", range_min);
    std::cout << "scan: min:" << range_min << std::endl;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        run();
        show_odom();
        show_scan();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw)
{
  tf::Quaternion quat(q.x,q.y,q.z,q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
