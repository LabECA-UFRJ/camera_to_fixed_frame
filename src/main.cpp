#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

ros::Publisher publisher;
string reference_frame_name;
string robot_frame_name;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle node;

    if (ros::param::get("~reference", reference_frame_name) == false)
    {
        ROS_FATAL("Parameter reference not set.");
        return -1;
    }

    if (ros::param::get("~robot", robot_frame_name) == false)
    {
        ROS_FATAL("Parameter reference not set.");
        return -1;
    }    

    publisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("position", 1000);    

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseStamped positionTransformed;

        try{
            transformStamped = tfBuffer.lookupTransform(reference_frame_name, robot_frame_name, ros::Time(0));
            
            positionTransformed.header = transformStamped.header;
            positionTransformed.pose.position.x = transformStamped.transform.translation.x;
            positionTransformed.pose.position.y = transformStamped.transform.translation.y;
            positionTransformed.pose.position.z = transformStamped.transform.translation.z;
            positionTransformed.pose.orientation.x = transformStamped.transform.rotation.x;
            positionTransformed.pose.orientation.y = transformStamped.transform.rotation.y;
            positionTransformed.pose.orientation.z = transformStamped.transform.rotation.z;
            positionTransformed.pose.orientation.w = transformStamped.transform.rotation.w;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        publisher.publish(positionTransformed);
        rate.sleep();
    }

    ros::spin();

    return 0;
}