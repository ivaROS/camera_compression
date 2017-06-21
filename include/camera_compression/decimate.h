#ifndef IMAGE_DECIMATION_H
#define IMAGE_DECIMATION_H

#include <opencv2/core/core.hpp>

namespace camera_compression
{

cv::Mat decimate(const cv::Mat & image, int decimation)
{
	ROS_ASSERT(decimation >= 1);
	cv::Mat out;
	if(!image.empty())
	{
		if(decimation > 1)
		{
			if((image.type() == CV_32FC1 || image.type()==CV_16UC1))
			{
				ROS_ERROR_STREAM_COND(image.rows % decimation == 0 && image.cols % decimation == 0,
						"Decimation of depth images should be exact! (decimation=" << decimation << ", size=" << 
						image.cols << "x" << image.rows );

				out = cv::Mat(image.rows/decimation, image.cols/decimation, image.type());
				if(image.type() == CV_32FC1)
				{
					for(int j=0; j<out.rows; ++j)
					{
						for(int i=0; i<out.cols; ++i)
						{
							out.at<float>(j, i) = image.at<float>(j*decimation, i*decimation);
						}
					}
				}
				else // CV_16UC1
				{
					for(int j=0; j<out.rows; ++j)
					{
						for(int i=0; i<out.cols; ++i)
						{
							out.at<unsigned short>(j, i) = image.at<unsigned short>(j*decimation, i*decimation);
						}
					}
				}
			}
			else
			{
				cv::resize(image, out, cv::Size(), 1.0f/float(decimation), 1.0f/float(decimation), cv::INTER_AREA);
			}
		}
		else
		{
			out = image;
		}
	}
	return out;
}

} //end namespace

#endif
