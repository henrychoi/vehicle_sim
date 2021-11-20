// #include <iostream>
#include <string>
#include <deque>
// #include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <ros/console.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Int32.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace std;

class ArucoPublisher {
	typedef ArucoPublisher Self;
public:
	ArucoPublisher();

private:
	void onFrame(const sensor_msgs::ImageConstPtr& msg);
	void onCam2Marker(const geometry_msgs::PoseStampedConstPtr& p);
	void onCameraInfo(const sensor_msgs::CameraInfo &msg);

    ros::NodeHandle _nh, _ph;

	/**
	 * Camera _intrinsic matrix pre initialized with _intrinsic values for the test camera.
	 */
	double data_calibration[9] = {
		640, 0, 360, 
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
	image_transport::Publisher debug_img_pub;
	image_transport::Subscriber _image0_sub, _image1_sub, _image2_sub, _image3_sub;
	ros::Subscriber _cal1_sub;

	tf2_ros::TransformBroadcaster _br;
	tf2_ros::Buffer _tf2_buffer;
	tf2_ros::TransformListener _tf2_listener;
	ros::Publisher _cam2marker_pub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> _cam2marker_sub;
	tf2_ros::MessageFilter<geometry_msgs::PoseStamped> _tf2_filter;

	/**
	 * Flag to check if _intrinsic parameters were received.
	 * If set to false the camera will be _calibrated when a camera info message is received.
	 */
	bool _calibrated = false;

	bool _show_axis = false;

	Ptr<aruco::GridBoard> _board;

	struct _CamState {
		Vec3d rvec, tvec;
	} camState_[4];
	struct _DetectionScore {
		int camId;
		vector<int> markerIds;
	};
	deque<_DetectionScore> detectedQ_; // will be at most 2 elements long


	struct _DetectionResult {
		bool valid;
		size_t score;
		geometry_msgs::Point T;
		tf2::Quaternion Q;
	} other_;
	// queue<_DetectionResult> resQ_;
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
void ArucoPublisher::onFrame(const sensor_msgs::ImageConstPtr& msg) {
	// ROS_INFO("onFrame %s", msg->header.frame_id.c_str());
	int camId = msg->header.frame_id[4] - '0';
	if (camId > 3) {
		ROS_INFO("Image from unexpected camera frame %s", msg->header.frame_id.c_str());
		return;
	}
	if (!_calibrated) {
		ROS_INFO("camera not yet calibrated camera frame %s", msg->header.frame_id.c_str());
		return;
	}
	try {
		auto t0 = ros::Time::now();
		Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		aruco::detectMarkers(frame, _board->dictionary, markerCorners, markerIds
			// optional args
			// detector parameters, rejectedImgPoints, _intrinsic, _distortion
			);
		if(markerIds.size() <= 0
			|| !aruco::estimatePoseBoard(markerCorners, markerIds, _board
					, _intrinsic, _distortion
					, camState_[camId].rvec, camState_[camId].tvec
					// sometimes yields Z axis going INTO the board. so can't use this
					// , cv::SOLVEPNP_P3P
					)) {
			return;
		}
		// Record the latest valid board observation time
		detectedQ_.push_back({camId, markerIds});
		auto elapsed = ros::Time::now() - t0;
		ROS_DEBUG("%zd markers in cam%u; R = [%.2f, %.2f, %.2f] T = [%.3f, %.3f, %.3f]"
				, markerIds.size(), camId
				, camState_[camId].rvec[0], camState_[camId].rvec[1], camState_[camId].rvec[2]
				, camState_[camId].tvec[0], camState_[camId].tvec[1], camState_[camId].tvec[2]);

		float angle = sqrt(camState_[camId].rvec[0] * camState_[camId].rvec[0]
						+ camState_[camId].rvec[1] * camState_[camId].rvec[1]
						+ camState_[camId].rvec[2] * camState_[camId].rvec[2]);
		float sina2 = sin(0.5f * angle);
		float scale = sina2 / angle;

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
		geometry_msgs::PoseStamped cam2marker;
		cam2marker.pose.orientation.x =  camState_[camId].rvec[2] * scale;
		cam2marker.pose.orientation.y = -camState_[camId].rvec[0] * scale;
		cam2marker.pose.orientation.z = -camState_[camId].rvec[1] * scale;
		cam2marker.pose.orientation.w = cos(0.5f * angle);
		cam2marker.pose.position.x =  camState_[camId].tvec[2];
		cam2marker.pose.position.y = -camState_[camId].tvec[0];
		cam2marker.pose.position.z = -camState_[camId].tvec[1];
		cam2marker.header.stamp = msg->header.stamp;
		cam2marker.header.frame_id = format("quad%d_link", camId);
		static uint32_t sSeq = 0;
		cam2marker.header.seq = ++sSeq;

		// save away the state of each camera
		_cam2marker_pub.publish(cam2marker);

		ROS_DEBUG("cam%u markers pose Q = [%.2f, %.2f, %.2f, %.2f]", camId
				, cam2marker.pose.orientation.x
				, cam2marker.pose.orientation.y
				, cam2marker.pose.orientation.z
				, cam2marker.pose.orientation.w);

		if (_show_axis) { // Show the board frame
		  	cv::aruco::drawAxis(frame, _intrinsic, _distortion
			  	, camState_[camId].rvec, camState_[camId].tvec, 0.8);
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			// preserve the timestamp from the image frame
			img->header.stamp = msg->header.stamp;
			debug_img_pub.publish(img);
		}
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Cannot get image");
	}
}

void ArucoPublisher::onCam2Marker(const geometry_msgs::PoseStampedConstPtr& cam2marker) {
	// ROS_INFO("onPose with frame %s", cam2marker->header.frame_id.c_str());
	int camId = cam2marker->header.frame_id[4] - '0';
    geometry_msgs::PoseStamped quad2marker;
    try {
    	_tf2_buffer.transform(*cam2marker, quad2marker, "quad_link");
		tf2::Quaternion Q;
		tf2::fromMsg(quad2marker.pose.orientation, Q);
		{
			const auto axis = Q.getAxis();
			ROS_DEBUG("cam %d pose in quad_link Q = [%.2f, %.2f, %.2f, %.2f] T = [%.3f, %.3f, %.3f]"
				, camId
				// confirmed that the camera's timestamps are the same (synchronized capture)
				//, quad2marker.header.stamp.sec, quad2marker.header.stamp.nsec
				// , quad2marker.pose.orientation.x
				// , quad2marker.pose.orientation.y
				// , quad2marker.pose.orientation.z
				// , quad2marker.pose.orientation.w
				, axis[0], axis[1], axis[2], Q.getW()
				, quad2marker.pose.position.x
				, quad2marker.pose.position.y
				, quad2marker.pose.position.z);
		}
		switch (detectedQ_.size()) {
			case 2: { // I shouldn't finish calculation; save away the result
				_DetectionScore& det = (detectedQ_.front().camId == camId)
					? detectedQ_.front() : detectedQ_.back();
				other_.score = det.markerIds.size();
				other_.T = quad2marker.pose.position;
				other_.Q = Q;
				other_.valid = true;
				if (detectedQ_.front().camId == camId) {
					detectedQ_.pop_front();
				} else {
					detectedQ_.pop_back();
				}
			}	break;

			case 1: { // average the current result with one stored in other_
				_DetectionScore& det = detectedQ_.front();
				auto score = static_cast<float>(det.markerIds.size());
				// pose from THIS camera
				auto T = quad2marker.pose.position;
				auto Qave = Q;
				if (other_.valid) { // weighted average with other camera's estimate
					auto weight = 1.f / (score + other_.score);

					// weighted average of the 2 translations
					T.x = weight * (score * T.x + other_.score * other_.T.x);
					T.y = weight * (score * T.y + other_.score * other_.T.y);
					T.z = weight * (score * T.z + other_.score * other_.T.z);
					Qave = other_.Q.slerp(Q, score*weight);
				}
				detectedQ_.clear();
				other_.valid = false; // consumed result

				geometry_msgs::TransformStamped transformStamped;
				transformStamped.header = quad2marker.header;
				transformStamped.child_frame_id = "aruco";
				transformStamped.transform.translation.x = T.x;
				transformStamped.transform.translation.y = T.y;
				transformStamped.transform.translation.z = T.z;
				transformStamped.transform.rotation.x = Qave.x();
				transformStamped.transform.rotation.y = Qave.y();
				transformStamped.transform.rotation.z = Qave.z();
				transformStamped.transform.rotation.w = Qave.w();
				_br.sendTransform(transformStamped);

				ROS_DEBUG("pose in quad_link Q = [%.2f, %.2f, %.2f, %.2f] T = [%.3f, %.3f, %.3f]"
						, transformStamped.transform.rotation.x
						, transformStamped.transform.rotation.y
						, transformStamped.transform.rotation.z
						, transformStamped.transform.rotation.w
						, transformStamped.transform.translation.x
						, transformStamped.transform.translation.y
						, transformStamped.transform.translation.z);
			}	break;

			default:
				ROS_ERROR("Precondition violation: detectedQ_ size %zd ! 1 or 2"
					, detectedQ_.size());
				detectedQ_.clear();
		}


	} catch (tf2::TransformException &ex) {
  	    ROS_ERROR("Transform failure %s\n", ex.what());
  	}
}
/**
 * On camera info callback.
 * Used to receive camera _intrinsic parameters.
 */
void ArucoPublisher::onCameraInfo(const sensor_msgs::CameraInfo &msg) {
	if(_calibrated) return;
	_calibrated = true;
	
	for(unsigned int i = 0; i < 9; i++)
		_intrinsic.at<double>(i / 3, i % 3) = msg.K[i];
	
	for(unsigned int i = 0; i < 5; i++)
		_distortion.at<double>(0, i) = msg.D[i];

	if(_show_axis) {
		ROS_INFO("frame %s instrinsic received", msg.header.frame_id.c_str());
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

ArucoPublisher::ArucoPublisher()
: _nh("aruco"), _ph("~")
, _it(_nh)
, debug_img_pub(_it.advertise("debug", 1))
, _image0_sub(_it.subscribe("quad0/image_raw", 1, &Self::onFrame, this))
, _image1_sub(_it.subscribe("quad1/image_raw", 1, &Self::onFrame, this))
, _image2_sub(_it.subscribe("quad2/image_raw", 1, &Self::onFrame, this))
, _image3_sub(_it.subscribe("quad3/image_raw", 1, &Self::onFrame, this))
, _cal1_sub(_nh.subscribe("quad3/camera_info", 1 , &Self::onCameraInfo, this))
, _cam2marker_pub(_ph.advertise<geometry_msgs::PoseStamped>("cam2marker", 1, true))
, _tf2_listener(_tf2_buffer)
, _tf2_filter(_cam2marker_sub, _tf2_buffer, "quad_link", 10, 0)
{
	int Nrows, Ncols;
	float length, gap;

	assert(_nh.param("Nrows", Nrows, 1));
	assert(_nh.param("Ncols", Ncols, 1));
	assert(_nh.param("length", length, 0.15f));
	assert(_nh.param("gap", gap, 0.01f));
	assert(_nh.param("show_axis", _show_axis, false));
	_nh.param("calibrated", _calibrated, false);

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

	_cam2marker_sub.subscribe(_ph, "cam2marker", 10);
  	_tf2_filter.registerCallback(boost::bind(&Self::onCam2Marker, this, _1));
  
  	auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	_board = aruco::GridBoard::create(Ncols, Nrows, length, gap, dict);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aruco");
	ArucoPublisher pub;
	ros::spin();

	return 0;
}
