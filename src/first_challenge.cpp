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

void FirstChallenge::run(float v, float r)
{


       cmd_vel_.mode = 11;
       cmd_vel_.cntl.linear.x = v;
       cmd_vel_.cntl.angular.z = r;

       pub_cmd_vel_.publish(cmd_vel_);

}

void FirstChallenge::show_odom()
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << " theta:" << tf::getYaw(odometry_.pose.pose.orientation) <<std::endl;
}

float FirstChallenge::show_scan()
{
    float range_min = 1e6;
    int size_medium = laser_.ranges.size()/2;
    int size_min = size_medium -20;
    int size_max = size_medium +20;

    for (int i = size_min; i < size_max; i++) {
        if (laser_.ranges[i] < range_min) {
            range_min = laser_.ranges[i];
        }
    }
    // ROS_INFO_STREAM("scan: min: %f", range_min);
    std::cout << "scan: min:" << range_min << std::endl;

    return range_min;
}

void FirstChallenge::process()
{

    moved = false;
    rotated = false;
    init_flag = false;
    stopped = false;

    int t = 0;
    float time = t*hz_;

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(moved != true){
            run(0.10, 0.00);
            ros::spinOnce();
            loop_rate.sleep();

            if(odometry_.pose.pose.orientation.x > 1.0){
                moved = true;
            }
        }                                                               //動き終わり
        else if(moved == true && rotated != true ){
            if(init_flag != true){
                goal = tf::getYaw(odometry_.pose.pose.orientation);
                init_flag = true;
            }
            else{
                run(0.0, 0.10);
                ros::spinOnce();
                loop_rate.sleep();
                t += 1;


                if(fabs(goal - tf::getYaw(odometry_.pose.pose.orientation)) < 0.05 && time > 5){
                        rotated = true;
                }
                                                                        //回転終わり
            }
        }
        else if(moved == true && rotated == true && stopped != true){
            run(0.10, 0.00);
            ros::spinOnce();
            loop_rate.sleep();

            if(show_scan() < 0.50){
                stopped = true;
            }
         }
        else if(moved == true && rotated == true && stopped == true){
            run(0.0, 0.0);
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
