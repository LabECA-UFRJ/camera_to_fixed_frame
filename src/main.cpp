#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

ros::Publisher publisher;
string reference_frame_name;
string robot_frame_name;

double lpf_coefficient;
geometry_msgs::PoseStamped lastPositionTransformed;
bool hasLastTrasform;

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

    if (ros::param::get("~low_pass_filter", lpf_coefficient) == false)
    {
        ROS_FATAL("Parameter low_pass_filter not set.");
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

            if (hasLastTrasform == false) {
                positionTransformed.header = transformStamped.header;
                positionTransformed.pose.position.x = transformStamped.transform.translation.x;
                positionTransformed.pose.position.y = transformStamped.transform.translation.y;
                positionTransformed.pose.position.z = transformStamped.transform.translation.z;
                positionTransformed.pose.orientation.x = transformStamped.transform.rotation.x;
                positionTransformed.pose.orientation.y = transformStamped.transform.rotation.y;
                positionTransformed.pose.orientation.z = transformStamped.transform.rotation.z;
                positionTransformed.pose.orientation.w = transformStamped.transform.rotation.w;

                lastPositionTransformed = positionTransformed;
                hasLastTrasform = true;
            }
            else { 
                positionTransformed.header = transformStamped.header;

                // General formula -> y[i] := α * x[i] + (1-α) * y[i-1] where 0 <= α <= 1
                positionTransformed.pose.position.x = lpf_coefficient * (transformStamped.transform.translation.x) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.position.x);
                positionTransformed.pose.position.y = lpf_coefficient * (transformStamped.transform.translation.y) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.position.y);
                positionTransformed.pose.position.z = lpf_coefficient * (transformStamped.transform.translation.z) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.position.z);
                positionTransformed.pose.orientation.x = lpf_coefficient * (transformStamped.transform.rotation.x) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.orientation.x);
                positionTransformed.pose.orientation.y = lpf_coefficient * (transformStamped.transform.rotation.y) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.orientation.y);
                positionTransformed.pose.orientation.z = lpf_coefficient * (transformStamped.transform.rotation.z) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.orientation.z);
                positionTransformed.pose.orientation.w = lpf_coefficient * (transformStamped.transform.rotation.w) + (1 - lpf_coefficient)*(lastPositionTransformed.pose.orientation.w);

                lastPositionTransformed = positionTransformed;
            }

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