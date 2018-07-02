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
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_transport/subscriber_filter.h"
#include "sensor_msgs/Image.h"

namespace activity_recognition
{
	class ActivityRecognition
	{
		private:
			ros::NodeHandle nh_;
			ros::Subscriber op_sub_;

			typedef image_transport::SubscriberFilter ImageSubscriber;
			typedef message_filters::Subscriber<sensor_msgs::Image> image_sub;
			typedef message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> open_pose_sub;
			image_sub rgb_image_sub_;
			image_sub depth_image_sub_;
			open_pose_sub open_pose_sub_;

			cv_bridge::CvImagePtr cv_img_ptr_;
			cv_bridge::CvImagePtr cv_depth_ptr_;
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, openpose_ros_msgs::OpenPoseHumanList> MySyncPolicy;
			message_filters::Synchronizer<MySyncPolicy> *sync;
			
			
			void syncTopics();
			void poseRecieved(const sensor_msgs::ImageConstPtr& img_msg,
                              const sensor_msgs::ImageConstPtr& depth_msg,
							  const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& op_msg);

			human human1;
		public:
			ActivityRecognition();
			virtual ~ActivityRecognition();
			void recognizeActivity();
	};
}
