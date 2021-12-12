#include <string>
#include <deque>

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

#include <cmath>

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
	ros::Subscriber cal0_sub_, cal2_sub_;

	tf2_ros::TransformBroadcaster _br;
	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener _tf2_listener;
	ros::Publisher _cam2marker_pub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> _cam2marker_sub;
	tf2_ros::MessageFilter<geometry_msgs::PoseStamped> _tf2_filter;

	ros::Publisher aruco_tf_strobe_;
	/**
	 * Flag to check if _intrinsic parameters were received.
	 * If set to false the camera will be _calibrated when a camera info message is received.
	 */
	bool _calibrated = false;

	bool _show_axis = false;

	Ptr<aruco::GridBoard> board_;

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

static const tf2::Quaternion sQx180(1, 0, 0, 0)
, sQy180(0, 1, 0, 0)
, sQcv2ros(0.5, 0.5, 0.5, 0.5) // = (0, 0, sqrt(2), sqrt(2)) x (sqrt(2), 0, 0, sqrt(2))
, sQaruco2ros(0.5, 0.5, -0.5, -0.5) // = (0, 0, sqrt(2), sqrt(2)) x (-sqrt(2), 0, 0, sqrt(2))
;

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
		aruco::detectMarkers(frame, board_->dictionary, markerCorners, markerIds
			// optional args
			// detector parameters, rejectedImgPoints, _intrinsic, _distortion
			);
		if(markerIds.size() <= 0
			|| !aruco::estimatePoseBoard(markerCorners, markerIds, board_
					, _intrinsic, _distortion
					, camState_[camId].rvec, camState_[camId].tvec
					// sometimes yields Z axis going INTO the board. so can't use this
					// , cv::SOLVEPNP_P3P
					)) {
			return;
		}

		string markerIdStr = format("%d", markerIds[0]);
		for (auto i=1; i < markerIds.size(); ++i) {
			markerIdStr += format(",%d", markerIds[i]);
		}
		// output rotation vector is an angle * axis formulation
		// cvRodrigues2() converts rotation vector to to a 3-by-3 rotation matrix  

		// Record the latest valid board observation score
		detectedQ_.push_back({camId, markerIds});
		auto elapsed = ros::Time::now() - t0;
		float angle = sqrt(camState_[camId].rvec[0] * camState_[camId].rvec[0]
						+ camState_[camId].rvec[1] * camState_[camId].rvec[1]
						+ camState_[camId].rvec[2] * camState_[camId].rvec[2]);
		float sina2 = sin(0.5f * angle);
		float scale = sina2 / angle;

		ROS_INFO("markers (%s) in cam%u; T = [%.2f, %.2f, %.2f] R = [%.2f, %.2f, %.2f]"
				" "
				, markerIdStr.c_str(), camId
				, camState_[camId].tvec[0], camState_[camId].tvec[1], camState_[camId].tvec[2]
				, camState_[camId].rvec[0], camState_[camId].rvec[1], camState_[camId].rvec[2]
				);

		/* Publish TF note the flipping from CV --> ROS
			Units should be in meters and radians. The OpenCV uses
			Z+ to represent depth, Y- for height and X+ for lateral,
			but ROS uses X+ for depth (axial), Z+ for height, and
			Y- for lateral movement.
				
				ROS		Z+			OpenCV
						|          
						|    X+      Z+					Y+
						|    /       /					|
						|   /       /					|
						|  /       /					|
						| /       /						|
						|/       /						|
		Y+ -------------O		O-------------> X+		O-------------> X+
						:		|					   /
						:		|					  /
						:		|					 /
						:		|					/
						Z-		Y+				   Z+
		*/
		tf2::Quaternion Q(camState_[camId].rvec[2] * scale
						, -camState_[camId].rvec[0] * scale
						, -camState_[camId].rvec[1] * scale
						, cos(0.5f * angle));
		Q *= sQx180; // right multiply the flipping of the aruco axis

		geometry_msgs::PoseStamped cam2marker;
		// rotate the quaternion formed above about x-axis by 180 deg
		cam2marker.pose.orientation.x = Q.x();
		cam2marker.pose.orientation.y = Q.y();
		cam2marker.pose.orientation.z = Q.z();
		cam2marker.pose.orientation.w = Q.w();
		cam2marker.pose.position.x =  camState_[camId].tvec[2];
		cam2marker.pose.position.y = -camState_[camId].tvec[0];
		cam2marker.pose.position.z = -camState_[camId].tvec[1];
		cam2marker.header.stamp = msg->header.stamp;
		cam2marker.header.frame_id = format("quad%d_link", camId);
		static uint32_t sSeq = 0;
		cam2marker.header.seq = ++sSeq;

		// save away the state of each camera
		_cam2marker_pub.publish(cam2marker);

		// yaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
		// pitch = asin(-2.0*(qx*qz - qw*qy));
		// roll = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
		tf2::Vector3 axis = Q.getAxis();
		ROS_INFO(//"%d.%03u "
			"cam%u (%.2f, %.2f); Q(%.2f, %.2f, %.2f, %.2f) = [%.2f, %.2f, %.2f] %.2f"
			// , msg->header.stamp.sec, msg->header.stamp.nsec/1000000
			, camId, cam2marker.pose.position.x, cam2marker.pose.position.y
			, Q.x(), Q.y(), Q.z(), Q.w(), axis[0], axis[1], axis[2], Q.getAngle());
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
    	tf2_buffer_.transform(*cam2marker, quad2marker, "quad_link");
		tf2::Quaternion Q;
		tf2::fromMsg(quad2marker.pose.orientation, Q);
		ROS_DEBUG(//"%d.%03u "
			"cam %d [%.3f, %.3f; %.2f, %.2f, %.2f, %.2f] in quad_link"
			//, quad2marker.header.stamp.sec, quad2marker.header.stamp.nsec/1000000
			, camId
			, quad2marker.pose.position.x, quad2marker.pose.position.y
			, quad2marker.pose.orientation.x
			, quad2marker.pose.orientation.y
			, quad2marker.pose.orientation.z
			, quad2marker.pose.orientation.w
		);
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

				if (abs(Qave.length2() - 1.0) > 0.001) { // invalid transform
					ROS_ERROR("Invalid Qave norm");
					break;
				}
				tf2::Vector3 axis = Qave.getAxis();

				geometry_msgs::TransformStamped xf;
				xf.header = quad2marker.header;
				// Assume the vehicle ONLY yaws
				double yaw = 0;
				if (abs(axis[2]) > 0.8) { // rotation axis roughly along vertical
					// => can assume that the rotation amount is the 
					yaw = Qave.getAngle() * (-2*signbit(axis[2])+1);
				}

				xf.child_frame_id = "aruco";
				xf.transform.translation.x = T.x;
				xf.transform.translation.y = T.y;
				xf.transform.translation.z = T.z;
				xf.transform.rotation.x = 0; //Qave.x();
				xf.transform.rotation.y = 0; //Qave.y();
				xf.transform.rotation.z = sin(0.5 * yaw); //Qave.z();
				xf.transform.rotation.w = cos(0.5 * yaw); //Qave.w();
				_br.sendTransform(xf);

				ROS_DEBUG(//"%d.%03u "
					"aruco <-- quad_link = [%.2f, %.2f; (%.2f, %.2f, %.2f), %.2f]"
					//, xf.header.stamp.sec, xf.header.stamp.nsec/1000000
					, xf.transform.translation.x, xf.transform.translation.y
					, axis[0], axis[1], axis[2], yaw);
				// tell tf2 listeners that there is a new tf2 update
				aruco_tf_strobe_.publish(xf.header);

				if (false) { 
					auto xform = tf2_buffer_.lookupTransform("trailer", "base_link"
						, ros::Time(0)
						//, xf.header.stamp // can't lookup by the same timestamp
						);
					tf2::Quaternion q(xform.transform.rotation.x
						, xform.transform.rotation.y
						, xform.transform.rotation.z
						, xform.transform.rotation.w)
						;
					const auto axi = q.getAxis();
					ROS_INFO("trailer -> base_link = [%.2f, %.2f; (%.2f, %.2f, %.2f), %.2f]"
						, xform.transform.translation.x, xform.transform.translation.y
						, axi[0], axi[1], axi[2], q.getAngle());
				}
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
, _image0_sub(_it.subscribe("/quad0/image_raw", 1, &Self::onFrame, this))
, _image1_sub(_it.subscribe("/quad1/image_raw", 1, &Self::onFrame, this))
, _image2_sub(_it.subscribe("/quad2/image_raw", 1, &Self::onFrame, this))
, _image3_sub(_it.subscribe("/quad3/image_raw", 1, &Self::onFrame, this))
, cal0_sub_(_nh.subscribe("/quad0/camera_info", 1 , &Self::onCameraInfo, this))
, cal2_sub_(_nh.subscribe("/quad2/camera_info", 1 , &Self::onCameraInfo, this))
, _cam2marker_pub(_ph.advertise<geometry_msgs::PoseStamped>("cam2marker", 1, true))
, _tf2_listener(tf2_buffer_)
, _tf2_filter(_cam2marker_sub, tf2_buffer_, "quad_link", 10, 0)
, aruco_tf_strobe_(_nh.advertise<std_msgs::Header>("tf_strobe", 1, true))
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
	board_ = aruco::GridBoard::create(Ncols, Nrows, length, gap, dict);
	assert(board_->ids.size() == board_->objPoints.size());
	for (auto i=0; i < board_->objPoints.size(); ++i) {
		const auto& rect = board_->objPoints[i];
		ROS_INFO("rectangle: %d", board_->ids[i]);
		for (auto pt: rect) {
			ROS_INFO("%s", format("(%.2f, %.2f, %.2f)", pt.x, pt.y, pt.z).c_str());
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aruco");
	ArucoPublisher pub;
	ros::spin();

	return 0;
}
