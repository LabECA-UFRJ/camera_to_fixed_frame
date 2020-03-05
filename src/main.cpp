#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

bool receivedFixedMarker = false;

tf2::Vector3 referencePosition;
tf2::Quaternion referenceRotation;

tf2::Vector3 sendingPosition;
tf2::Quaternion sendingRotation;

ros::Publisher publisher;

void referenceCallback(const geometry_msgs::Pose::ConstPtr& referenceData)
{
    receivedFixedMarker = true;

    fromMsg(referenceData->position, referencePosition);
    fromMsg(referenceData->orientation, referenceRotation);
}

void inPoseCallback(const geometry_msgs::Pose::ConstPtr& receivedData)
{
    if (receivedFixedMarker == false)
        return;

    tf2::Vector3 temporaryPosition; 
    tf2::Quaternion temporaryRotation;
    
    fromMsg(receivedData->position, temporaryPosition);
    fromMsg(receivedData->orientation, temporaryRotation);

    temporaryPosition = referencePosition - temporaryPosition;
    sendingPosition = quatRotate(referenceRotation, temporaryPosition);

    sendingRotation = referenceRotation.inverse() * temporaryRotation;

    geometry_msgs::Point positionMessage = toMsg(sendingPosition, positionMessage);
    geometry_msgs::Quaternion quaternionMessage = toMsg(sendingRotation);

    geometry_msgs::Pose fullMessage;
    fullMessage.position = positionMessage;
    fullMessage.orientation = quaternionMessage;

    publisher.publish(fullMessage);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_to_fixed_frame");
    ros::NodeHandle nodeHandle;

    ros::Subscriber referenceSubscriber = nodeHandle.subscribe("reference_pose", 1000, referenceCallback);

    ros::Subscriber inputSubscriber = nodeHandle.subscribe("in_pose", 1000, inPoseCallback);

    publisher = nodeHandle.advertise<geometry_msgs::Pose>("out_pose", 1000);

    ros::spin();

    return 0;
}