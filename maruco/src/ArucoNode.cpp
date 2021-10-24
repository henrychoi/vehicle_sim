// #include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/photo/photo.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h> //#include <std_msgs/msg/string.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>//#include <sensor_msgs/image.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <maruco/Marker.h>

#include "ArucoMarker.cpp"
#include "ArucoMarkerInfo.cpp"
#include "ArucoDetector.cpp"

using namespace cv;
using namespace std;

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

/**
 * Node visibility publisher.
 * Publishes true when a known marker is visible, publishes false otherwise.
 */
ros::Publisher pub_visible;

/**
 * Node position publisher.
 */
ros::Publisher pub_position;

/**
 * Node rotation publisher.
 */
ros::Publisher pub_rotation;

/**
 * Node pose publisher.
 */
ros::Publisher pub_pose;

/**
 * Node odometry publisher.
 * Publishes the odometry of the tf_frame indicated using the pose calculated from marker.
 */
ros::Publisher pub_odom;

image_transport::Publisher pub_debug_img;

/**
 * Name of the transform tf name to indicate on published topics.
 */
string tf_frame_id;

/**
 * Pose publisher sequence counter.
 */
int pub_pose_seq = 0;

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
void onFrame(const sensor_msgs::ImageConstPtr& msg)
{
	// ROS_INFO("onFrame %u.%09u", msg->header.stamp.sec, msg->header.stamp.nsec);
	try
	{
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
		{
			for(unsigned int j = 0; j < known.size(); j++)
			{
				if(markers[i].id == known[j].id)
				{
					markers[i].attachInfo(known[j]);
					
					for(unsigned int k = 0; k < 4; k++)
					{
						projected.push_back(markers[i].projected[k]);
						world.push_back(known[j].world[k]);
					}

					found.push_back(markers[i]);
				}
			}
		}

		//Draw markers
		if(debug)
		{
			ArucoDetector::drawMarkers(frame, markers, calibration, distortion);
		}

		//Check if any marker was found
		if(world.size() > 0)
		{
			//Calculate position and rotation
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

			//Publish position and rotation
			geometry_msgs::Point message_position, message_rotation;

			//Robot coordinates
			message_position.x = camera_position.at<double>(2, 0);
			message_position.y = -camera_position.at<double>(0, 0);
			message_position.z = -camera_position.at<double>(1, 0);
			
			message_rotation.x = camera_rotation.at<double>(2, 0);
			message_rotation.y = -camera_rotation.at<double>(0, 0);
			message_rotation.z = -camera_rotation.at<double>(1, 0);

			pub_position.publish(message_position);
			pub_rotation.publish(message_rotation);

			//Publish pose
			geometry_msgs::PoseStamped message_pose;

			//Header
			message_pose.header.frame_id = tf_frame_id;
			message_pose.header.seq = pub_pose_seq++;
			message_pose.header.stamp = ros::Time::now();

			//Position
			message_pose.pose.position.x = message_position.x;
			message_pose.pose.position.y = message_position.y;
			message_pose.pose.position.z = message_position.z;

			//Convert to quaternion
			double x = message_rotation.x;
			double y = message_rotation.y;
			double z = message_rotation.z;

			//Module of angular velocity
			double angle = sqrt(x*x + y*y + z*z);
			if(angle > 0.0)
			{
				auto sa = sin(angle/2.0);
				message_pose.pose.orientation.x = x * sa/angle;
				message_pose.pose.orientation.y = y * sa/angle;
				message_pose.pose.orientation.z = z * sa/angle;
				message_pose.pose.orientation.w = cos(angle/2.0);
			}
			//To avoid illegal expressions
			else
			{
				message_pose.pose.orientation.x = 0.0;
				message_pose.pose.orientation.y = 0.0;
				message_pose.pose.orientation.z = 0.0;
				message_pose.pose.orientation.w = 1.0;
			}
			
			pub_pose.publish(message_pose);

            nav_msgs::Odometry message_odometry;
            message_odometry.header.frame_id = tf_frame_id;
            message_odometry.header.stamp = ros::Time::now();
            message_odometry.pose.pose = message_pose.pose;
            pub_odom.publish(message_odometry);

			if(debug) {
				ArucoDetector::drawOrigin(frame, found, calibration, distortion, 0.1);
				auto img = boost::make_shared<sensor_msgs::Image>();
				convert_frame_to_message(frame, img);
				img->header.stamp = msg->header.stamp; // preserve the timestamp
				pub_debug_img.publish(img);
			}
		}

		//Publish visible
		std_msgs::Bool message_visible;
		message_visible.data = world.size() != 0;
		pub_visible.publish(message_visible);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Error getting image data");
	}
}

/**
 * On camera info callback.
 * Used to receive camera calibration parameters.
 */
void onCameraInfo(const sensor_msgs::CameraInfo &msg)
{
	if(!calibrated)
	{
		calibrated = true;
		
		for(unsigned int i = 0; i < 9; i++)
		{
			calibration.at<double>(i / 3, i % 3) = msg.K[i];
		}
		
		for(unsigned int i = 0; i < 5; i++)
		{
			distortion.at<double>(0, i) = msg.D[i];
		}

		if(debug)
		{
			cout << "Camera calibration param received" << endl;
			cout << "Camera: " << calibration << endl;
			cout << "Distortion: " << distortion << endl;
		}
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

/**
 * Main method launches aruco ros node, the node gets image and calibration parameters from camera, and publishes position and rotation of the camera relative to the markers.
 * Units should be in meters and radians, the markers are described by a position and an euler rotation.
 * Position is also available as a pose message that should be easier to consume by other ROS nodes.
 * Its possible to pass markers as arugment to this node or register and remove them during runtime using another ROS node.
 * The coordinate system used by OpenCV uses Z+ to represent depth, Y- for height and X+ for lateral, but for the node the coordinate system used is diferent X+ for depth, Z+ for height and Y- for lateral movement.
 * The coordinates are converted on input and on output, its possible to force the OpenCV coordinate system by setting the use_opencv_coords param to true.
 *   
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
 *
 * @param argc Number of arguments.
 * @param argv Value of the arguments.
 */
int main(int argc, char **argv)
{
	if (argc < 2) {
		ROS_FATAL("node name required");
		return -1;
	}
	ros::init(argc, argv, argv[1]);

	//ROS node instance
	ros::NodeHandle node(argv[1]);
	
	//Parameters
	node.param<bool>("debug", debug, false);
	node.param<float>("cosine_limit", cosine_limit, 0.7);
	node.param<int>("theshold_block_size_min", theshold_block_size_min, 3);
	node.param<int>("theshold_block_size_max", theshold_block_size_max, 21);
	node.param<float>("max_error_quad", max_error_quad, 0.035); 
	node.param<int>("min_area", min_area, 100);
	node.param<bool>("calibrated", calibrated, false);

	//Initial threshold block size
	theshold_block_size = (theshold_block_size_min + theshold_block_size_max) / 2;
	if(theshold_block_size % 2 == 0)
	{
		theshold_block_size++;
	}

	//Camera instrinsic calibration parameters
	if(node.hasParam("calibration"))
	{
		string data;
		node.param<string>("calibration", data, "");
		
		double values[9];
		stringToDoubleArray(data, values, 9, "_");

		for(unsigned int i = 0; i < 9; i++)
		{
			calibration.at<double>(i / 3, i % 3) = values[i];
		}

		calibrated = true;
	}
#if 0
	//Camera distortion calibration parameters
	if(node.hasParam("distortion"))
	{
		string data;
		node.param<string>("distortion", data, "");	

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
		if(node.hasParam("marker" + to_string(i))) {
			string data;
			node.param<string>("marker" + to_string(i), data, "1_0_0_0_0_0_0");

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

    // TF frame
    node.param<string>("tf_frame_id", tf_frame_id, "robot");

	image_transport::ImageTransport it(node);

	pub_visible = node.advertise<std_msgs::Bool>("visible", 10);
	pub_position = node.advertise<geometry_msgs::Point>("position", 10);
	pub_rotation = node.advertise<geometry_msgs::Point>("rotation", 10);
	pub_pose = node.advertise<geometry_msgs::PoseStamped>("pose", 10);
    pub_odom = node.advertise<nav_msgs::Odometry>("odom", 10);
    pub_debug_img = it.advertise("debug", 1);

	image_transport::Subscriber sub_camera = it.subscribe("image_raw", 1, onFrame);
	ros::Subscriber sub_camera_info = node.subscribe("camera_info", 1, onCameraInfo);

	ros::spin();

	return 0;
}
