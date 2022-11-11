#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <image_geometry/pinhole_camera_model.h>

/******************************Define***********************************/
#define ARUCO_SINGLE_MARKER		1
#define ARUCO_MARKER_SIZE		1.0


using namespace std;
using namespace sensor_msgs;
using namespace cv;

class ArucoDetector
{
private:
	ros::NodeHandle nh;

	/******************************Subscriber***********************************/
	ros::Subscriber cam_info_sub;
	image_transport::Subscriber image_sub;

	/******************************Publisher***********************************/
	ros::Publisher pose_pub;
	image_transport::Publisher image_pub;

	/******************************Variable***********************************/
	image_transport::ImageTransport it;
	image_geometry::PinholeCameraModel camera_model;

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	cv::Mat inputImage;

	cv::Ptr<cv::aruco::Dictionary> dictionary;

	bool ReceivedCamInfo;

public:
	/*
	 * Constructor
	 */
	ArucoDetector() : nh("~"), it(nh)
	{
		image_sub = it.subscribe("/iris/c920/image_raw", 1, &ArucoDetector::image_callback, this);

		cam_info_sub = nh.subscribe("/iris/c920/camera_info", 1, &ArucoDetector::cam_info_callback, this);

		image_pub = it.advertise("result", 1);

		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

		/*
		 *Init params camera info
		 */
		cameraMatrix = Mat(3, 3, CV_64FC1);
		distCoeffs = Mat(1, 5, CV_64F);

		int i;
		for (i = 0; i < 5; i++) {
			distCoeffs.at<double>(0,i) = 0.0;
		}

		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if ((image_pub.getNumSubscribers() == 0) && (pose_pub.getNumSubscribers() == 0)) {

			ROS_DEBUG("No subscribers, not looking for ArUco markers");

			return;
		}

		if (ReceivedCamInfo) {

			ros::Time CurrentTimeStamp = msg->header.stamp;

			cv_bridge::CvImagePtr ptrCVBridge;

			try {
				ptrCVBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inputImage = ptrCVBridge->image;

				std::vector<std::vector<cv::Point2f>> markerCorners;
				std::vector<int> markerIds;

				cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

				// If at least one marker detected
				if(markerIds.size() > 0) {

					cv::Mat rvec, tvec;
					cv::aruco::estimatePoseSingleMarkers(markerCorners,
														ARUCO_MARKER_SIZE,
														cameraMatrix,
														distCoeffs,
														rvec,
														tvec);

					geometry_msgs::PoseStamped poseMsg;

					poseMsg.header.frame_id = "camera_ref";
					poseMsg.header.stamp = CurrentTimeStamp;

					poseMsg.pose.position.x = tvec.at<double>(0);
					poseMsg.pose.position.y = tvec.at<double>(1);
					poseMsg.pose.position.z = tvec.at<double>(2);

					pose_pub.publish(poseMsg);

					if (image_pub.getNumSubscribers() > 0) {
						cv::drawFrameAxes(inputImage, cameraMatrix, distCoeffs, rvec, tvec, 1.0, 2);

						cv_bridge::CvImage outImage_msg;

						outImage_msg.header.stamp = CurrentTimeStamp;
						outImage_msg.encoding = sensor_msgs::image_encodings::RGB8;
						outImage_msg.image = inputImage;

						image_pub.publish(outImage_msg.toImageMsg());
					}
				}

			} catch (cv_bridge::Exception& e) {

				ROS_ERROR("cv_bridge exception: %s", e.what());

				return;
			}
		}
	}

	/*
	 * Wait for one camerainfo, then shut down that subscriber
	 */
	void cam_info_callback(const sensor_msgs::CameraInfo &info_msg)
	{
		// cameraMatrix.at<double>(0, 0) = msg.K[0];
		// cameraMatrix.at<double>(0, 1) = msg.K[1];
		// cameraMatrix.at<double>(0, 2) = msg.K[2];
		// cameraMatrix.at<double>(1, 0) = msg.K[3];
		// cameraMatrix.at<double>(1, 1) = msg.K[4];
		// cameraMatrix.at<double>(1, 2) = msg.K[5];
		// cameraMatrix.at<double>(2, 0) = msg.K[6];
		// cameraMatrix.at<double>(2, 1) = msg.K[7];
		// cameraMatrix.at<double>(2, 2) = msg.K[8];

		camera_model.fromCameraInfo(info_msg);

		distCoeffs = cv::Mat(camera_model.distortionCoeffs());

		cameraMatrix = cv::Mat(camera_model.fullIntrinsicMatrix());

		if (distCoeffs.empty()) {
			distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
		}

		ReceivedCamInfo = true;
		ROS_INFO("Get cameraMaxtrix done!!!");
		cam_info_sub.shutdown();
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_detect");

	ArucoDetector ArucoDetector;

	ROS_INFO("Aruco detect is running");

	ros::spin();
}
