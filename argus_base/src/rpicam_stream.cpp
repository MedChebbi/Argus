#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rpicam_stream_cpp");
    ros::NodeHandle nh;  // Default handler for nodes in ROS

    // 0 reads from your default camera
    const int CAMERA_INDEX = 0;
    cv::VideoCapture capture(CAMERA_INDEX,cv::CAP_V4L);
    if (!capture.isOpened()) {
      ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
      ros::shutdown();
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH,640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    capture.set(cv::CAP_PROP_BRIGHTNESS,-8);
    // Image_transport is responsible for publishing and subscribing to Images
    image_transport::ImageTransport it(nh);
     
    // Publish to the /camera topic
    image_transport::Publisher pub_frame = it.advertise("camera/image_raw", 1);

    cv::Mat frame;//Mat is the image class defined in OpenCV
    cv::Mat image;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(20);

    while (nh.ok()) {

      // Load image
      capture >> frame;

      // Check if grabbed frame has content
      if (frame.empty()) {
        ROS_ERROR_STREAM("Failed to capture image!");
        ros::shutdown();
      }
      // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
      //flip(frame,image,-1);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub_frame.publish(msg);
      //cv::imshow("camera", frame);
      //cv::waitKey(1); // Display image for 1 millisecond

      ros::spinOnce();
      loop_rate.sleep();
    }

    // Shutdown the camera
    capture.release();
}
