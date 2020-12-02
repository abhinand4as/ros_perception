
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "pubsub/MsgTemplate.h"


void pointCallback(const geometry_msgs::Point::ConstPtr& point) {

    ROS_INFO_STREAM_THROTTLE(5, "Point: x=" << point->x << "\ty=" << point->y << "\tz=" << point->z);    

}


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "subscriber");

    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    spinner.start();



    ros::Subscriber point_sub = n.subscribe("pubsub/point", 1000, pointCallback);


    ros::waitForShutdown();

    return 0;
}