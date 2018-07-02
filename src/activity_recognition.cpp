#include "activity_recognition_ros/activity_recognition.h"


using namespace activity_recognition;

ActivityRecognition::ActivityRecognition()
{    
    //op_sub_ = nh_.subscribe("openpose_ros/human_list", 1, &ActivityRecognition::poseRecieved, this);

    syncTopics();

    cv::namedWindow("Activity Recognition");
    cv::namedWindow("Activity Recognition2");    
}
ActivityRecognition::~ActivityRecognition(){
    cv::destroyWindow("Activity Recognition");
    cv::destroyWindow("Activity Recognition2");
}


void ActivityRecognition::recognizeActivity()
{
    //ROS_INFO_STREAM("Hello func\n");
    cv::circle(human1.bgr_image, cv::Point(human1.joints[4][0],human1.joints[4][1]), 40, cv::Scalar(0, 0, 255), 8, 8);
    cv::circle(human1.bgr_image, cv::Point(human1.joints[7][0],human1.joints[7][1]), 40, cv::Scalar(0, 0, 255), 8, 8); 


    ROS_INFO_STREAM( "\n" << int(human1.depth_image.at<uchar>(human1.joints[4][0], human1.joints[4][1])) << "\n"); 
    

    cv::circle(human1.depth_image, cv::Point(human1.joints[4][0],human1.joints[4][1]), 40, cv::Scalar(255,255,255), 8, 8);
    cv::circle(human1.depth_image, cv::Point(human1.joints[7][0],human1.joints[7][1]), 40, cv::Scalar(255,255,255), 8, 8); 

    cv::imshow("Activity Recogntion", human1.bgr_image);
    cv::imshow("Activity Recogntion2", human1.depth_image);
    cv::waitKey(3);
    
    /*ROS_INFO_STREAM("\n" << "Right Hand" << human1.joints[4][0]<< " " <<
                            human1.joints[4][1] << " " <<
                            human1.joints[4][2] << "\n");
    */
}


/* Callback function for open pose data
void ActivityRecognition::poseRecieved(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg)
{
    /*
    openpose_ros_msgs::OpenPoseHumanList human_list_msg = *msg;
    std::vector<openpose_ros_msgs::OpenPoseHuman> human_list = human_list_msg.human_list;
    openpose_ros_msgs::OpenPoseHuman first_human = human_list[0];

    int num_body_key_points_with_non_zero_prob = first_human.num_body_key_points_with_non_zero_prob;

    ROS_INFO_STREAM("Joint 0 x: " << first_human.body_key_points_with_prob[0].x << "\n"
                    "Joint 0 y: " << first_human.body_key_points_with_prob[0].y << "\n"
                    "Joint 0 prob: " << first_human.body_key_points_with_prob[0].prob << "\n");
   */
    /*
    openpose_ros_msgs::OpenPoseHumanList human_list_msg = *msg;
    std::vector<openpose_ros_msgs::OpenPoseHuman> human_list = human_list_msg.human_list;
    openpose_ros_msgs::OpenPoseHuman first_human = human_list[0];

    for (int i = 0; i < 18; i++){
        human1.joints[i][0] = first_human.body_key_points_with_prob[i].x;
        human1.joints[i][1] = first_human.body_key_points_with_prob[i].y;
        human1.joints[i][2] = (first_human.body_key_points_with_prob[i].prob * 100);
    }
}
*/
/*
void ActivityRecognition::imageRecieved(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    human1.bgr_image = (cv_img_ptr_->image);
}

void ActivityRecognition::depthRecieved(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    human1.depth_image = (cv_depth_ptr_->image);
}
*/

void ActivityRecognition::syncTopics()
{
    rgb_image_sub_.subscribe(nh_, "/camera/rgb/image_raw", 1 );
    depth_image_sub_.subscribe(nh_, "/camera/depth/image_raw", 1 );
    open_pose_sub_.subscribe(nh_,"/openpose_ros/human_list",1);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy( 100 ), rgb_image_sub_,  depth_image_sub_,open_pose_sub_);
    sync->registerCallback( boost::bind( &ActivityRecognition::poseRecieved, this, _1, _2, _3 ) );
}



void ActivityRecognition::poseRecieved(const sensor_msgs::ImageConstPtr& img_msg,
                                       const sensor_msgs::ImageConstPtr& depth_msg,
                                       const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& op_msg)
{
    /*
    openpose_ros_msgs::OpenPoseHumanList human_list_msg = *msg;
    std::vector<openpose_ros_msgs::OpenPoseHuman> human_list = human_list_msg.human_list;
    openpose_ros_msgs::OpenPoseHuman first_human = human_list[0];

    int num_body_key_points_with_non_zero_prob = first_human.num_body_key_points_with_non_zero_prob;

    ROS_INFO_STREAM("Joint 0 x: " << first_human.body_key_points_with_prob[0].x << "\n"
                    "Joint 0 y: " << first_human.body_key_points_with_prob[0].y << "\n"
                    "Joint 0 prob: " << first_human.body_key_points_with_prob[0].prob << "\n");
   
    */
    
    openpose_ros_msgs::OpenPoseHumanList human_list_msg = *op_msg;
    std::vector<openpose_ros_msgs::OpenPoseHuman> human_list = human_list_msg.human_list;
    openpose_ros_msgs::OpenPoseHuman first_human = human_list[0];

    for (int i = 0; i < 18; i++){
        human1.joints[i][0] = first_human.body_key_points_with_prob[i].x;
        human1.joints[i][1] = first_human.body_key_points_with_prob[i].y;
        human1.joints[i][2] = (first_human.body_key_points_with_prob[i].prob * 100);
    }
    
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    human1.bgr_image = (cv_img_ptr_->image);   

    
    
    try
    {
        cv_depth_ptr_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    human1.depth_image = (cv_depth_ptr_->image);
}
