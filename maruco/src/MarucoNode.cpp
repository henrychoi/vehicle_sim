// #include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

class MarucoPublisher {
public:
	MarucoPublisher(const char* suffix);

private:
	void onFrame(const sensor_msgs::ImageConstPtr& msg);
	void onCameraInfo(const sensor_msgs::CameraInfo &msg);

    ros::NodeHandle _nh;
	string _name_prefix;
	string link_name = "aruco_";

	/**
	 * Camera _intrinsic matrix pre initialized with _intrinsic values for the test camera.
	 */
	double data_calibration[9] = {
		640, 0, 640, 
		0, 	640, 360,
		0, 0, 1
	};
	Mat _intrinsic = cv::Mat(3, 3, CV_64F, data_calibration);

	/**
	 * Lenses _distortion matrix initialized with values for the test camera.
	 */
	double data_distortion[5] = {0, 0, 0, 0, 0};
	Mat _distortion = cv::Mat(1, 5, CV_64F, data_distortion);

	image_transport::ImageTransport _it;
	tf::TransformBroadcaster broadcaster;
	image_transport::Publisher pub_debug_img;
	image_transport::Subscriber sub_camera;
	ros::Subscriber sub_camera_info;

	/**
	 * Flag to check if _intrinsic parameters were received.
	 * If set to false the camera will be _calibrated when a camera info message is received.
	 */
	bool _calibrated = false;

	bool _show_axis = false;

	Ptr<aruco::GridBoard> _board;
	Vec3d _rvec, _tvec;
};

/*
 * CV array type to ROS sensor_msgs/Image type
 * https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
 */
const char* //std::string
mat_type2encoding(int mat_type) {
  switch (mat_type) {
    case CV_8UC1: return "mono8";
    case CV_8UC3: return "bgr8";
    case CV_16SC1: return "mono16";
    case CV_8UC4: return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
  }
}

static void convert_frame_to_message(const cv::Mat& frame,
	boost::shared_ptr<sensor_msgs::Image> msg) {
  // copy cv information into ros message
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast<sensor_msgs::Image::_step_type>(frame.step);
  const auto size = frame.step * frame.rows;
#if 1 // pointer swap, using C array as an iterator
  msg->data.assign(frame.data, frame.data+size);
#else // memcpy
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size); // blit the whole image to ROS msg
#endif
  //msg->header.frame_id = std::to_string(frame_id);
//   msg->header.frame_id = "base_link";
}

/**
 * Callback executed every time a new camera frame is received.
 * This callback is used to process received images and publish messages with camera position data if any.
 */
void MarucoPublisher::onFrame(const sensor_msgs::ImageConstPtr& msg) {
	// ROS_INFO("onFrame");// %u.%09u", msg->header.stamp.sec, msg->header.stamp.nsec);
	if (!_calibrated) {
		ROS_INFO("camera not yet calibrated");
		return;
	}
	try {
		auto t0 = ros::Time::now();
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		aruco::detectMarkers(frame, _board->dictionary, markerCorners, markerIds
			// optional args
			// detector parameters, rejectedImgPoints, _intrinsic, _distortion
			);
		if(markerIds.size() <= 0
			|| !aruco::estimatePoseBoard(markerCorners, markerIds, _board
					, _intrinsic, _distortion, _rvec, _tvec)) {
			return;
		}
		float angle = sqrt(_rvec[0]*_rvec[0] + _rvec[1]*_rvec[1] + _rvec[2]*_rvec[2]);
		tf::Vector3 axis(_rvec[2], -_rvec[0], _rvec[1]); // CV --> ROS
		tf::Quaternion q(axis, angle);
		auto elapsed = ros::Time::now() - t0;
		ROS_INFO("%d nsec; %zd markers; Q = [%.2f, %.2f, %.2f, %.2f] T = [%.3f, %.3f, %.3f]"
				, elapsed.nsec, markerIds.size()
				, q[0], q[1], q[2], q[3], _tvec[0], _tvec[1], _tvec[2]);
		if (_show_axis) { // Show the board frame
		  	cv::aruco::drawAxis(frame, _intrinsic, _distortion, _rvec, _tvec, 0.8);
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			// preserve the timestamp from the image frame
			img->header.stamp = msg->header.stamp;
			pub_debug_img.publish(img);
		}

#if 0
		/* Publish TF note the flipping from CV --> ROS
			Units should be in meters and radians. The OpenCV uses
			Z+ to represent depth, Y- for height and X+ for lateral, but ROS uses X+ for depth (axial), Z+ for height, and
			Y- for lateral movement.
		*           ROS          |          OpenCV
		*    Z+                  |    Y- 
		*    |                   |    |
		*    |    X+             |    |    Z+
		*    |    /              |    |    /
		*    |   /               |    |   /
		*    |  /                |    |  /
		*    | /                 |    | /
		*    |/                  |    |/
		*    O-------------> Y-  |    O-------------> X+
		 */
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(
			camera_position.at<double>(2, 0) // CV Z
			, -camera_position.at<double>(0, 0) // -CV X
			, -camera_position.at<double>(1, 0) // -CV Y
			) ); 
		tf::Quaternion q;
		// geometry_msgs::Point message_position, message_rotation;
		// Convert Euler angles to quaternion
		double ex = camera_rotation.at<double>(2, 0)
			, ey = -camera_rotation.at<double>(0, 0)
			, ez = -camera_rotation.at<double>(1, 0)
			, angle = sqrt(ex*ex + ey*ey + ez*ez);
		if(angle > 0.0) {
			auto sa = sin(angle/2.0);
			q[0] = ex * sa/angle;
			q[1] = ey * sa/angle;
			q[2] = ez * sa/angle;
			q[3] = cos(angle/2.0);
		} else { //To avoid illegal expressions
			ROS_ERROR("Failed to convert Euler(%.2f,%.2f,%.2f) --> quat"
			 	, ex, ey, ez);
			q[0] = q[1] = q[2] = 0.0;
			q[3] = 1.0;
		}
		transform.setRotation(q);
		broadcaster.sendTransform(tf::StampedTransform(transform
			, msg->header.stamp, "base_link", link_name));
#endif
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Error getting image data");
	}
}

/**
 * On camera info callback.
 * Used to receive camera _intrinsic parameters.
 */
void MarucoPublisher::onCameraInfo(const sensor_msgs::CameraInfo &msg) {
	if(_calibrated) return;
	_calibrated = true;
	
	for(unsigned int i = 0; i < 9; i++)
		_intrinsic.at<double>(i / 3, i % 3) = msg.K[i];
	
	for(unsigned int i = 0; i < 5; i++)
		_distortion.at<double>(0, i) = msg.D[i];

	if(_show_axis) {
		cout << "Camera _intrinsic param received" << endl;
		cout << "Intrinsic: " << _intrinsic << endl;
		cout << "Distortion: " << _distortion << endl;
	}
}

/**
 * Converts a string with numeric values separated by a delimiter to an array of double values.
 * If 0_1_2_3 and delimiter is _ array will contain {0, 1, 2, 3}.
 * @param data String to be converted
 * @param values Array to store values on
 * @param cout Number of elements in the string
 * @param delimiter Separator element
 * @return Array with values.
 */
void stringToDoubleArray(string data, double* values, unsigned int count, string delimiter)
{
	unsigned int pos = 0, k = 0;

	while((pos = data.find(delimiter)) != string::npos && k < count)
	{
		string token = data.substr(0, pos);
		values[k] = stod(token);
		data.erase(0, pos + delimiter.length());
		k++;
	}
}

MarucoPublisher::MarucoPublisher(const char* suffix)
: _nh("")
, _name_prefix(string("aruco_") + suffix + "/")
, _it(_nh)
, pub_debug_img(_it.advertise(_name_prefix +"debug", 1))
, sub_camera(_it.subscribe(string("cam_") + suffix + "/image_raw"
	, 1, &MarucoPublisher::onFrame, this))
, sub_camera_info(_nh.subscribe(string("cam_") + suffix + "/camera_info"
	, 1 , &MarucoPublisher::onCameraInfo, this))
{
	const auto ns = _nh.getNamespace();
	link_name += suffix;

	int Nrows, Ncols;
	float length, gap;

	assert(_nh.param(_name_prefix + "Nrows", Nrows, 1));
	assert(_nh.param(_name_prefix + "Ncols", Ncols, 1));
	assert(_nh.param(_name_prefix + "length", length, 0.15f));
	assert(_nh.param(_name_prefix + "gap", gap, 0.01f));
	assert(_nh.param(_name_prefix + "show_axis", _show_axis, false));
	_nh.param(_name_prefix + "calibrated", _calibrated, false);

	// assert(_show_axis);

	//Camera instrinsic _intrinsic parameters
	if(_nh.hasParam("calibration")) {
	#if 1 // TODO ROS param can be a vector of double too!
	#else
		string data;
		_nh.param<string>("calibration", data, "");
		
		double values[9];
		stringToDoubleArray(data, values, 9, "_");

		for(unsigned int i = 0; i < 9; i++)
		{
			_intrinsic.at<double>(i / 3, i % 3) = values[i];
		}
	#endif
	}

#if 0
	//Camera _distortion _intrinsic parameters
	if(_nh.hasParam("distortion"))
	{
		string data;
		_nh.param<string>("distortion", data, "");	

		double values[5];
		stringToDoubleArray(data, values, 5, "_");

		for(unsigned int i = 0; i < 5; i++)
		{
			_distortion.at<double>(0, i) = values[i];
		}

		_calibrated = true;
	}
#endif
	auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	_board = aruco::GridBoard::create(Ncols, Nrows, length, gap, dict);
}

int main(int argc, char **argv) {
	if (argc < 2) {
		fprintf(stderr, "aruco camera location {fl, fr, rl, rr} required");
		return -1;
	}
	argc = 0;
	ros::init(argc, NULL, string("aruco_") + argv[1]);
	MarucoPublisher pub(argv[1]);
	ros::spin();

	return 0;
}