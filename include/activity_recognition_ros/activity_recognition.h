#include "ros/ros.h"

#include "openpose_ros_msgs/BoundingBox.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"

#include "activity_recognition_ros/human.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_transport/subscriber_filter.h"


namespace activity_recognition
{
	class ActivityRecognition
	{
		private:
			ros::NodeHandle nh_;
			ros::Subscriber op_sub_;
			
			image_transport::ImageTransport it_;
			image_transport::Subscriber image_sub_;
			image_transport::Subscriber depth_sub_;

			cv_bridge::CvImagePtr cv_img_ptr_;
			cv_bridge::CvImagePtr cv_depth_ptr_;

			void poseRecieved(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg);
			void imageRecieved(const sensor_msgs::ImageConstPtr& msg);
			void depthRecieved(const sensor_msgs::ImageConstPtr& msg);		

			//void ActivityRecognition::poseRecieved(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& op_msg,
            //                            const sensor_msgs::ImageConstPtr& img_msg,
            //                            const sensor_msgs::ImageConstPtr& depth_msg);

			human human1;
		public:
			ActivityRecognition();
			virtual ~ActivityRecognition();
			void recognizeActivity();
	};
}
