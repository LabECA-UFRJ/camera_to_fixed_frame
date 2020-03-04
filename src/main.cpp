#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

using namespace std;

geometry_msgs::Pose fixedMarkerPose;
bool receivedFixedMarker = false;

struct Vector3 {
    float x;
    float y;
    float z;
};

struct Quat {
    float x;
    float y;
    float z;
    float w;
};

Vector3 fixedMarkerPosition;
Quat fixedMarkerRotation;

Vector3 newPosition;
Quat newRotation;

void setPositionAndRotation(const geometry_msgs::Pose::ConstPtr& receivedData, Vector3 position, Quat rotation)
{
    position.x = receivedData->Point->x;
    position.y = receivedData->Point->y;
    position.z = receivedData->Point->z;

    rotation.x = receivedData->Quaternion->x;
    rotation.y = receivedData->Quaternion->y;
    rotation.z = receivedData->Quaternion->z;
    rotation.w = receivedData->Quaternion->w;
}

void fixedMarkerCallback(const geometry_msgs::Pose::ConstPtr& fixedMarkerData)
{
    fixedMarkerPose = fixedMarkerData;
    receivedFixedMarker = true;

    setPositionAndRotation(fixedMarkerData, fixedMarkerPosition, fixedMarkerRotation);
}

void arucoCallback(const geometry_msgs::Pose::ConstPtr& arucoData)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_to_fixed_frame");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("fixed_marker", 1000, fixedMarkerCallback);

    ros::Subscriber subscriber = nodeHandle.subscribe("aruco", 1000, arucoCallback);

    if (receivedFixedMarker == true)
        ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Pose>("fixed_frame", 1000);

    ros::spin();

    return 0;
}