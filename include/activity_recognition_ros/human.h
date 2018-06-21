#include "opencv2/core/core.hpp"

namespace activity_recognition
{
	class human
	{
	public:
		human();
		virtual ~human();
		int joints[18][3];
		cv::Mat bgr_image = (cv::Mat::zeros(cv::Size(1,49), CV_64FC1));
		cv::Mat depth_image = (cv::Mat::zeros(cv::Size(1,49), CV_16UC1));
	private:
		/* data */
	};
	
}
