// #include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include "ArucoMarker.cpp"
#include "ArucoMarkerInfo.cpp"
#include "ArucoDetector.cpp"

using namespace cv;
using namespace std;

class MarucoPublisher {
public:
	MarucoPublisher(const char* suffix);

private:
	void onFrame(const sensor_msgs::ImageConstPtr& msg);
	void onCameraInfo(const sensor_msgs::CameraInfo &msg);

    ros::NodeHandle _nh;
	string link_name = "maruco_";

	/**
	 * Camera calibration matrix pre initialized with calibration values for the test camera.
	 */
	double data_calibration[9] = {640, 0, 360, 0, 640, 360, 0, 0, 1};
	Mat calibration = cv::Mat(3, 3, CV_64F, data_calibration);

	/**
	 * Lenses distortion matrix initialized with values for the test camera.
	 */
	double data_distortion[5] = {0, 0, 0, 0, 0};
	Mat distortion = cv::Mat(1, 5, CV_64F, data_distortion);

	/**
	 * List of known of markers, to get the absolute position and rotation of the camera, some of these are required.
	 */
	vector<ArucoMarkerInfo> known = vector<ArucoMarkerInfo>();

	image_transport::ImageTransport _it;
	tf::TransformBroadcaster broadcaster;
	image_transport::Publisher pub_debug_img;
	image_transport::Subscriber sub_camera;
	ros::Subscriber sub_camera_info;

	/**
	 * Flag to check if calibration parameters were received.
	 * If set to false the camera will be calibrated when a camera info message is received.
	 */
	bool calibrated;

	bool debug;

	/**
	 * Cosine limit used during the quad detection phase.
	 * Value between 0 and 1.
	 * By default 0.8 is used.
	 * The bigger the value more distortion tolerant the square detection will be.
	 */
	float cosine_limit;

	/**
	 * Maximum error to be used by geometry poly aproximation method in the quad detection phase.
	 * By default 0.035 is used.
	 */
	float max_error_quad;

	/**
	 * Adaptive theshold pre processing block size.
	 */
	int theshold_block_size;

	/**
	 * Minimum threshold block size.
	 * By default 5 is used.
	 */
	int theshold_block_size_min;

	/**
	 * Maximum threshold block size.
	 * By default 9 is used.
	 */
	int theshold_block_size_max;

	/**
	 * Minimum area considered for aruco markers.
	 * Should be a value high enough to filter blobs out but detect the smallest marker necessary.
	 * By default 100 is used.
	 */
	int min_area;
};

/**
 * Draw yellow text with black outline into a frame.
 * @param frame Frame mat.
 * @param text Text to be drawn into the frame.
 * @param point Position of the text in frame coordinates.
 */
void drawText(Mat frame, string text, Point point) {
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, 0, 2, LINE_AA);
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, 255, 1, LINE_AA);
}

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
	try {
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		//Process image and get markers
		vector<ArucoMarker> markers = ArucoDetector::getMarkers(frame, cosine_limit, theshold_block_size, min_area, max_error_quad);

		//Visible
		vector<ArucoMarker> found;

		//Vector of points
		vector<Point2f> projected;
		vector<Point3f> world;

		if(markers.size() == 0)
		{
			theshold_block_size += 2;

			if(theshold_block_size > theshold_block_size_max)
			{
				theshold_block_size = theshold_block_size_min;
			}
		}

		//Check known markers and build known of points
		for(unsigned int i = 0; i < markers.size(); i++)
			for(unsigned int j = 0; j < known.size(); j++)
				if(markers[i].id == known[j].id) {
					markers[i].attachInfo(known[j]);
					
					for(unsigned int k = 0; k < 4; k++)
					{
						projected.push_back(markers[i].projected[k]);
						world.push_back(known[j].world[k]);
					}

					found.push_back(markers[i]);
				}

		//Draw markers
		if(debug) {
			ArucoDetector::drawMarkers(frame, markers, calibration, distortion);
		}

		//Check if any marker was found
		if(!world.size()) {
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			img->header.stamp = msg->header.stamp; // preserve the timestamp
			pub_debug_img.publish(img); //throw up the debug img
			return; // and bail
		}

		// Calculate position and rotation from CV PNP
		Mat rotation, position;
		#if CV_MAJOR_VERSION == 2
			solvePnP(world, projected, calibration, distortion, rotation, position, false, ITERATIVE);
		#else
			solvePnP(world, projected, calibration, distortion, rotation, position, false, SOLVEPNP_ITERATIVE);
		#endif

		//Invert position and rotation to get camera coords
		Mat rodrigues;
		Rodrigues(rotation, rodrigues);
		
		Mat camera_rotation;
		Rodrigues(rodrigues.t(), camera_rotation);
		
		Mat camera_position = -rodrigues.t() * position;

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

		if(debug) {
			ArucoDetector::drawOrigin(frame, found, calibration, distortion, 0.1);
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			// preserve the timestamp from the image frame
			img->header.stamp = msg->header.stamp;
			pub_debug_img.publish(img);
		}
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Error getting image data");
	}
}

/**
 * On camera info callback.
 * Used to receive camera calibration parameters.
 */
void MarucoPublisher::onCameraInfo(const sensor_msgs::CameraInfo &msg) {
	if(calibrated) return;
	calibrated = true;
	
	for(unsigned int i = 0; i < 9; i++)
		calibration.at<double>(i / 3, i % 3) = msg.K[i];
	
	for(unsigned int i = 0; i < 5; i++)
		distortion.at<double>(0, i) = msg.D[i];

	if(debug) {
		cout << "Camera calibration param received" << endl;
		cout << "Camera: " << calibration << endl;
		cout << "Distortion: " << distortion << endl;
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
, _it(_nh)
, pub_debug_img(_it.advertise(string("maruco_")+suffix+"/debug", 1))
, sub_camera(_it.subscribe(string("cam_") + suffix + "/image_raw"
	, 1, &MarucoPublisher::onFrame, this))
, sub_camera_info(_nh.subscribe(string("cam_") + suffix + "/camera_info"
	, 1 , &MarucoPublisher::onCameraInfo, this))
{
	link_name += suffix;

	_nh.param<bool>("debug", debug, false);
	_nh.param<float>("cosine_limit", cosine_limit, 0.7);
	_nh.param<int>("theshold_block_size_min", theshold_block_size_min, 3);
	_nh.param<int>("theshold_block_size_max", theshold_block_size_max, 21);
	_nh.param<float>("max_error_quad", max_error_quad, 0.035); 
	_nh.param<int>("min_area", min_area, 100);
	_nh.param<bool>("calibrated", calibrated, false);

	//Initial threshold block size
	theshold_block_size = (theshold_block_size_min + theshold_block_size_max) / 2;
	if(theshold_block_size % 2 == 0)
		theshold_block_size++;

	//Camera instrinsic calibration parameters
	if(_nh.hasParam("calibration")) {
		string data;
		_nh.param<string>("calibration", data, "");
		
		double values[9];
		stringToDoubleArray(data, values, 9, "_");

		for(unsigned int i = 0; i < 9; i++)
		{
			calibration.at<double>(i / 3, i % 3) = values[i];
		}
	}

#if 0
	//Camera distortion calibration parameters
	if(_nh.hasParam("distortion"))
	{
		string data;
		_nh.param<string>("distortion", data, "");	

		double values[5];
		stringToDoubleArray(data, values, 5, "_");

		for(unsigned int i = 0; i < 5; i++)
		{
			distortion.at<double>(0, i) = values[i];
		}

		calibrated = true;
	}
#endif

	//Aruco makers passed as parameters
	for(unsigned int i = 0; i < 2; i++) {
		if(_nh.hasParam("marker" + to_string(i))) {
			string data;
			_nh.param<string>("marker" + to_string(i), data, "1_0_0_0_0_0_0");

			double values[7];
			stringToDoubleArray(data, values, 7, "_");
			known.push_back(ArucoMarkerInfo(i, values[0]
				, Point3d(-values[2], -values[3], -values[1])
				, Point3d(-values[5], -values[6], values[4])));
		}
	}

	if(debug) { //Print all known markers
		for(unsigned int i = 0; i < known.size(); i++)
			known[i].print();
	}
}

int main(int argc, char **argv) {
	if (argc < 2) {
		fprintf(stderr, "maruco camera location {fl, fr, rl, rr} required");
		return -1;
	}
	argc = 0;
	ros::init(argc, NULL, string("maruco_") + argv[1]);
	MarucoPublisher pub(argv[1]);
	ros::spin();

	return 0;
}
