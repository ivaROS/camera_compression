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
				ROS_ERROR_STREAM_COND(image.rows % decimation  || image.cols % decimation ,
						"Decimation of depth images should be exact! (decimation=" << decimation << ", size=" << 
						image.cols << "x" << image.rows << ")" );
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

template<typename T>//, uint decimation>
void decimate(const T* data_in, T* data_out, int rows, int cols, int decimation)
{
  for(int ri=0, ro=0; ro<rows; ri+=decimation, ++ro)
  {
    for(int ci=0, co=0; co<cols; ci+=decimation, ++co)
    {
      data_out[ro*cols+co] = data_in[ri*cols + ci];
    }
  }
}

template<typename T, int decimation>//, uint decimation>
void decimate(const T* data_in, T* data_out, int rows, int cols)
{
  for(int ri=0, ro=0; ro<rows; ri+=decimation, ++ro)
  {
    for(int ci=0, co=0; co<cols; ci+=decimation, ++co)
    {
      data_out[ro*cols+co] = data_in[ri*cols + ci];
    }
  }
}

sensor_msgs::Image::Ptr decimate(const sensor_msgs::Image::ConstPtr msg_in, int decimation)
{
  sensor_msgs::Image::Ptr msg_out = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image& out = *msg_out;
  
//   cv_bridge::CvImage::ConstPtr imagePtr = cv_bridge::toCvShare(msg_in);
//   const cv::Mat& image = imagePtr->image;
  
  out.header = msg_in->header;
  out.encoding = msg_in->encoding;
  out.is_bigendian = msg_in->is_bigendian;
  
  int in_rows = msg_in->height;
  int in_cols = msg_in->width;
  
  int out_rows = in_rows/decimation;
  int out_cols = in_cols/decimation;
  
  
  out.height = out_rows;
  out.width = out_cols;
  out.step = out_cols * msg_in->step/in_cols;
  size_t size = out.step * out_rows;
  
  out.data.resize(size);
  
  if(decimation==2)
  {
    decimate<float,2>((const float*)msg_in->data.data(), (float*)out.data.data(), out_rows, out_cols);
  }
  else
  {
    decimate<float>((const float*)msg_in->data.data(), (float*)out.data.data(), out_rows, out_cols, decimation);
  }
  
  return msg_out;
}

} //end namespace

#endif
