/* A ROS package to connect a network camera and publish frames over a ROS Topic.
* This ROS package connects to a network camera, whose details can be set between Line 42 and 44,
* grabs the frames of the camera and publishes over a ROS topic called /network camera as well as visualizing the frames on a OpenCV window.
* Make sure IP address, and the login credential of the webserver is correct.
* Line 42 and 43 contains the IP address of the network camera of interest.
* Line 44 contains the login credentials of the webserver.
* Maintainer: Murat Ambarkutuk (github.com/eroniki)
* Computational Multi-physics Systems Laboratory Virginia Tech, Blacksburg, VA
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <time.h>

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

struct cameraObject{
	std::string ip;
	std::string port;
	std::string streamURL;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "network_streamer");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher imagePublisher = it.advertise("/networkCam", 1);
  cameraObject networkCamera;

  networkCamera.ip = "http://192.168.1.6";
  networkCamera.port = "81";
  networkCamera.streamURL = "/videostream.cgi?loginuse=admin&loginpas=12345&dummy=param.mjpg";

  cv::namedWindow("Network Stream",1);
  std::string fullURL = networkCamera.ip + ":" + networkCamera.port + networkCamera.streamURL;
  std::cout<<"URL to stream from: "<<fullURL<<std::endl;
  cv::VideoCapture capture(fullURL);
  std::cout<<"capture object initialized"<<std::endl;

  cv::Mat frame;
  std::cout<<"frame created"<<std::endl;

  cv_bridge::CvImage rosImage;

	rosImage.encoding = "bgr8";
	rosImage.header.stamp = ros::Time::now();
	rosImage.header.frame_id = "bar";
	std::cout<<"ros image initialized"<<std::endl;

  while(ros::ok()){
  	capture.read(frame);
  	if(frame.empty()){
  		std::cout<<"Frame is empty"<<std::endl;
  		continue;
  	}
  	else{
  		rosImage.image = frame.clone();
		imagePublisher.publish(rosImage.toImageMsg());
		cv::imshow("Network Stream",frame);
		if(cv::waitKey(30) >= 0)
			break;
  	}
  	ros::spinOnce();
  }

	return 0;
}
