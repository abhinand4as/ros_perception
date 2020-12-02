#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher labelImagePublisher_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    labelImagePublisher_ =it_.advertise("label_image",
								   1,
								   true);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    int h = cv_ptr->image.rows;
    int w = cv_ptr->image.cols;
    ROS_INFO_STREAM("h: " << h << "\tw: " << w);
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
    //Convert this message to a ROS sensor_msgs::Image message. 
    image_pub_.publish(cv_ptr->toImageMsg());

    cv::Mat label_im(h, w, CV_32SC1, cv::Scalar::all(0));
    cv::rectangle(label_im,
                        cv::Point(50,50),
                        cv::Point(300,300),
                        cv::Scalar::all(250), CV_FILLED);
    
    cv::circle(label_im, cv::Point(50, 50), 10, CV_RGB(255,0,0));


    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "detection_image";
    cvImage.encoding = sensor_msgs::image_encodings::TYPE_32SC1;
    cvImage.image = label_im;

    cv::imshow(OPENCV_WINDOW, label_im);
    cv::waitKey(3);  
    labelImagePublisher_.publish(*cvImage.toImageMsg());

    // (!publishLabelImage(label_im)) {
    // ROS_DEBUG("Label image has not been broadcasted.");
  }

//   publishLabelImage(const cv::Mat& labelImage)
//     {
//         if (labelImagePublisher_.getNumSubscribers() < 1)
//             return false;
//         cv_bridge::CvImage cvImage;
//         cvImage.header.stamp = ros::Time::now();
//         cvImage.header.frame_id = "detection_image";
//         cvImage.encoding = sensor_msgs::image_encodings::TYPE_32SC1;
//         cvImage.image = labelImage;
//         labelImagePublisher_.publish(*cvImage.toImageMsg());
//         ROS_DEBUG("Label image has been published.");
//     return true;
//     }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}