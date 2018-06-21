#include "ros/ros.h"
#include "activity_recognition_ros/activity_recognition.h"


int main(int argc, char *argv[])
{
	// Initialize ros
	ros::init(argc, argv, "activity_recognition_ros");
	
	// Create activity recognition object; Initializes subscribers
	activity_recognition::ActivityRecognition ActivityRecognition;
	
	while (ros::ok()){
		ActivityRecognition.recognizeActivity();
		ros::spinOnce();
		//ROS_INFO_STREAM("Hello\n");
	}

	return 0;
}