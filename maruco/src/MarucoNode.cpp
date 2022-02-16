#include <string>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <ros/console.h>

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

static double pify(double alpha) {
  double v = fmod(alpha, 2*M_PI);
  if (v < -M_PI)
    v += 2*M_PI;
  else if (v > M_PI)
    v -= 2*M_PI;
  return v;
}

class Maruco {
	typedef Maruco Self;
public:
	Maruco();

private:
    ros::NodeHandle _nh, _ph;

    double _wheel_base, _wheel_tread, _wheel_radius; // params
	bool _show_axis = false;

	Mat _quadIntrinsic[4] = { cv::Mat(3, 3, CV_64F)
			, cv::Mat(3, 3, CV_64F), cv::Mat(3, 3, CV_64F), cv::Mat(3, 3, CV_64F)
		} , _monoIntrinsic = cv::Mat(3, 3, CV_64F) //, { 320,0,240, 0,320,240, 0,0,1})
		, _quadDistortion[4] = { cv::Mat(1, 5, CV_64F) //, {0, 0, 0, 0, 0})
			, cv::Mat(1, 5, CV_64F), cv::Mat(1, 5, CV_64F), cv::Mat(1, 5, CV_64F)
		}, _monoDistortion = cv::Mat(1, 5, CV_64F) //, {0, 0, 0, 0, 0})
		;
	image_transport::ImageTransport _it;
	image_transport::Publisher _debugImgPub;

	image_transport::Subscriber _quad0sub, _quad1sub, _quad2sub, _quad3sub;
	void onQuadFrame(const sensor_msgs::ImageConstPtr& msg);
	ros::Time _marker2QuadTime = ros::Time(0); // to reject redundant pose estimate
	const ros::Duration _markObsDeadlne;//(0.2);
	// x, y, for debouncing erroneous reading; assume the vehicle will not go directly
	// under the trailer (incorrect assumption if vehicle will park under the trailer)
	double _prevQuadPosition[2] = {0,0};

	image_transport::Subscriber _monosub;
	void onMonoFrame(const sensor_msgs::ImageConstPtr& msg);
	ros::Time _marker2MonoTime = ros::Time(0);
	double _prevMonoPosition[2] = {0,0};

	ros::Subscriber _quad0calSub, _quad1calSub, _quad2calSub, _quad3calSub;
	void onQuadCal(const sensor_msgs::CameraInfo &msg) {
		int id = msg.header.frame_id[4] - '0';
		if (id > 3) {
			ROS_INFO("Calibration from unexpected camera frame %s"
					, msg.header.frame_id.c_str());
			return;
		}
		if (_quadIntrinsic[id].at<double>(0,0)) {
			return;
		}
		for(unsigned int i = 0; i < 9; i++)
			_quadIntrinsic[id].at<double>(i / 3, i % 3) = msg.K[i];
		
		for(unsigned int i = 0; i < 5; i++)
			_quadDistortion[id].at<double>(0, i) = msg.D[i];

		if(_show_axis) {
			ROS_WARN("quad %d instrinsic received", id);
			cout << "Intrinsic: " << _quadIntrinsic[id] << endl;
			cout << "Distortion: " << _quadDistortion[id] << endl;
		}
	}

	ros::Subscriber _monoCalSub;
	void onMonoCal(const sensor_msgs::CameraInfo &msg) {
		if (_monoIntrinsic.at<double>(0,0)) {
			return;
		}
		for(unsigned int i = 0; i < 9; i++)
			_monoIntrinsic.at<double>(i / 3, i % 3) = msg.K[i];
		
		for(unsigned int i = 0; i < 5; i++)
			_monoDistortion.at<double>(0, i) = msg.D[i];

		if(_show_axis) {
			ROS_WARN("%s instrinsic received", msg.header.frame_id.c_str());
			cout << "Intrinsic: " << _monoIntrinsic << endl;
			cout << "Distortion: " << _monoDistortion << endl;
		}
	}

	tf2_ros::TransformBroadcaster _br;
	tf2_ros::Buffer tf2_buffer_;
	tf2_ros::TransformListener _tf2_listener;
	ros::Publisher _cam2marker_pub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> _cam2marker_sub;
	tf2_ros::MessageFilter<geometry_msgs::PoseStamped> _tf2_filter;

	ros::Publisher aruco_tf_strobe_;

	Ptr<aruco::Board> _markers, _btmMarkers;
	Ptr<aruco::DetectorParameters> _arucoDetectionParam = aruco::DetectorParameters::create();

	struct _CamState { Vec3d rvec = {0,0,0}, tvec = {0,0,0}; }
		_quadState[4], _monoState;

	struct BicycleState_ { ros::Time time; double k, ds; //, abs_ds;
	};
	deque<BicycleState_> _bicycleQ;
};


Maruco::Maruco()
: _nh("maruco"), _ph("~")
, _it(_nh)
, _debugImgPub(_it.advertise("/aruco/debug", 1))
, _quad0sub(_it.subscribe("/quad0/image_raw", 1, &Self::onQuadFrame, this))
, _quad1sub(_it.subscribe("/quad1/image_raw", 1, &Self::onQuadFrame, this))
, _quad2sub(_it.subscribe("/quad2/image_raw", 1, &Self::onQuadFrame, this))
, _quad3sub(_it.subscribe("/quad3/image_raw", 1, &Self::onQuadFrame, this))
, _quad0calSub(_nh.subscribe("/quad0/camera_info", 1 , &Self::onQuadCal, this))
, _quad1calSub(_nh.subscribe("/quad1/camera_info", 1 , &Self::onQuadCal, this))
, _quad2calSub(_nh.subscribe("/quad2/camera_info", 1 , &Self::onQuadCal, this))
, _quad3calSub(_nh.subscribe("/quad3/camera_info", 1 , &Self::onQuadCal, this))
, _monosub(_it.subscribe("/webcam/image_raw", 1, &Self::onMonoFrame, this))
, _monoCalSub(_nh.subscribe("/webcam/camera_info", 1 , &Self::onMonoCal, this))
, _markObsDeadlne(0.1) // > primary camera 1/FPS
, _cam2marker_pub(_ph.advertise<geometry_msgs::PoseStamped>("cam2marker", 1, true))
, _tf2_listener(tf2_buffer_)
, _tf2_filter(_cam2marker_sub, tf2_buffer_, "quad_link", 10, 0)
, aruco_tf_strobe_(_nh.advertise<std_msgs::Header>("/aruco/tf_strobe", 1, true))
{
	assert(_nh.param("show_axis", _show_axis, false));
    _nh.param("wheel_base", _wheel_base, 0.267);
    _nh.param("wheel_radius", _wheel_radius, 0.06);
    _nh.param("wheel_tread", _wheel_tread, 0.23); // wheel_tread = 0.5 * track width

#if 0
	//Camera instrinsic _quadIntrinsic[id] parameters
	if(_nh.hasParam("calibration")) {
		string data;
		_nh.param<string>("calibration", data, "");
		
		double values[9];
		stringToDoubleArray(data, values, 9, "_");

		for(unsigned int i = 0; i < 9; i++)
			_quadIntrinsic[id].at<double>(i / 3, i % 3) = values[i];
	}

	//Camera _quadDistortion[id] _quadIntrinsic[id] parameters
	if(_nh.hasParam("distortion"))
	{
		string data;
		_nh.param<string>("distortion", data, "");	

		double values[5];
		stringToDoubleArray(data, values, 5, "_");

		for(unsigned int i = 0; i < 5; i++)
			_quadDistortion[id].at<double>(0, i) = values[i];
	}
#endif

#if 0
  	// @see https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
	// _arucoDetectionParam->adaptiveThreshWinSizeMin *= 4;
	// _arucoDetectionParam->adaptiveThreshWinSizeMax *= 4;
	// _arucoDetectionParam->adaptiveThreshWinSizeStep *= 4;

  	_arucoDetectionParam->minMarkerPerimeterRate *= 5;
	// _arucoDetectionParam->minMarkerDistanceRate *= 2;
	_arucoDetectionParam->polygonalApproxAccuracyRate *= 0.5;
  	// _arucoDetectionParam->adaptiveThreshConstant *= 3;
#endif
  	auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
	vector<int> ids;
	for (auto i=0; i < 2*(6+8) + 12*16; ++i) { // 4 sides
		ids.push_back(i);
	}

	vector<vector<cv::Point3f>> corners;
	//front: 0~5
	corners.push_back({Point3f(	+0.244,		-0.143	,	+0.375)// top row
					, Point3f(	+0.091,		-0.143	,	+0.375)
					, Point3f(	+0.091,		-0.005	,	+0.375)
					, Point3f(	+0.244,		-0.005	,	+0.375)});
	corners.push_back({Point3f(	+0.080,		-0.143	,	+0.375)
					, Point3f(	-0.080,		-0.143	,	+0.375)
					, Point3f(	-0.080,		-0.005	,	+0.375)
					, Point3f(	+0.080,		-0.005	,	+0.375)});
	corners.push_back({Point3f(	-0.091,		-0.143	,	+0.375)
					, Point3f(	-0.244,		-0.143	,	+0.375)
					, Point3f(	-0.244,		-0.005	,	+0.375)
					, Point3f(	-0.091,		-0.005	,	+0.375)});
	corners.push_back({Point3f(	+0.244,		+0.005	,	+0.375)// bottom row
					, Point3f(	+0.091,		+0.005	,	+0.375)
					, Point3f(	+0.091,		+0.143	,	+0.375)
					, Point3f(	+0.244,		+0.143	,	+0.375)});
	corners.push_back({Point3f(	+0.080,		+0.005	,	+0.375)
					, Point3f(	-0.080,		+0.005	,	+0.375)
					, Point3f(	-0.080,		+0.143	,	+0.375)
					, Point3f(	+0.080,		+0.143	,	+0.375)});
	corners.push_back({Point3f(	-0.091,		+0.005	,	+0.375)
					, Point3f(	-0.244,		+0.005	,	+0.375)
					, Point3f(	-0.244,		+0.143	,	+0.375)
					, Point3f(	-0.091,		+0.143	,	+0.375)});
	// right: 6~13	
	corners.push_back({Point3f(	+0.255,		-0.143	,	-0.368	)// top row
					, Point3f(	+0.255,		-0.143	,	-0.193)
					, Point3f(	+0.255,		-0.005	,	-0.193)
					, Point3f(	+0.255,		-0.005	,	-0.368	)});
	corners.push_back({Point3f(	+0.255,		-0.143	,	-0.182)
					, Point3f(	+0.255,		-0.143	,	-0.006	)
					, Point3f(	+0.255,		-0.005	,	-0.006	)
					, Point3f(	+0.255,		-0.005	,	-0.182)});
	corners.push_back({Point3f(	+0.255,		-0.143	,	+0.006	)
					, Point3f(	+0.255,		-0.143	,	+0.182)
					, Point3f(	+0.255,		-0.005	,	+0.182)
					, Point3f(	+0.255,		-0.005	,	+0.006	)});
	corners.push_back({Point3f(	+0.255,		-0.143	,	+0.193)
					, Point3f(	+0.255,		-0.143	,	+0.368	)
					, Point3f(	+0.255,		-0.005	,	+0.368	)
					, Point3f(	+0.255,		-0.005	,	+0.193	)});
	corners.push_back({Point3f(	+0.255,		+0.005	,	-0.368	)// bottom row
					, Point3f(	+0.255,		+0.005	,	-0.193)
					, Point3f(	+0.255,		+0.143	,	-0.193)
					, Point3f(	+0.255,		+0.143	,	-0.368	)});
	corners.push_back({Point3f(	+0.255,		+0.005	,	-0.182)
					, Point3f(	+0.255,		+0.005	,	-0.006	)
					, Point3f(	+0.255,		+0.143	,	-0.006	)
					, Point3f(	+0.255,		+0.143	,	-0.182)});
	corners.push_back({Point3f(	+0.255,		+0.005	,	+0.006	)
					, Point3f(	+0.255,		+0.005	,	+0.182)
					, Point3f(	+0.255,		+0.143	,	+0.182)
					, Point3f(	+0.255,		+0.143	,	+0.006	)});
	corners.push_back({Point3f(	+0.255,		+0.005	,	+0.193)
					, Point3f(	+0.255,		+0.005	,	+0.368	)
					, Point3f(	+0.255,		+0.143	,	+0.368	)
					, Point3f(	+0.255,		+0.143	,	+0.193)});
	// rear: 143~19	
	corners.push_back({Point3f(	-0.244,		-0.143	,	-0.375)// top row
					, Point3f(	-0.091,		-0.143	,	-0.375)
					, Point3f(	-0.091,		-0.005	,	-0.375)
					, Point3f(	-0.244,		-0.005	,	-0.375)});
	corners.push_back({Point3f(	-0.080,		-0.143	,	-0.375)
					, Point3f(	+0.080,		-0.143	,	-0.375)
					, Point3f(	+0.080,		-0.005	,	-0.375)
					, Point3f(	-0.080,		-0.005	,	-0.375)});
	corners.push_back({Point3f(	+0.091,		-0.143	,	-0.375)
					, Point3f(	+0.244,		-0.143	,	-0.375)
					, Point3f(	+0.244,		-0.005	,	-0.375)
					, Point3f(	+0.091,		-0.005	,	-0.375)});
	corners.push_back({Point3f(	-0.244,		+0.005	,	-0.375)// bottom row
					, Point3f(	-0.091,		+0.005	,	-0.375)
					, Point3f(	-0.091,		+0.143	,	-0.375)
					, Point3f(	-0.244,		+0.143	,	-0.375)});
	corners.push_back({Point3f(	-0.080,		+0.005	,	-0.375)
					, Point3f(	+0.080,		+0.005	,	-0.375)
					, Point3f(	+0.080,		+0.143	,	-0.375)
					, Point3f(	-0.080,		+0.143	,	-0.375)});
	corners.push_back({Point3f(	+0.091,		+0.005	,	-0.375)
					, Point3f(	+0.244,		+0.005	,	-0.375)
					, Point3f(	+0.244,		+0.143	,	-0.375)
					, Point3f(	+0.091,		+0.143	,	-0.375)});
	// left: 20~27	
	corners.push_back({Point3f(	-0.255,		-0.143	,	+0.368)// top row
					, Point3f(	-0.255,		-0.143	,	+0.193)
					, Point3f(	-0.255,		-0.005	,	+0.193)
					, Point3f(	-0.255,		-0.005	,	+0.368)});
	corners.push_back({Point3f(	-0.255,		-0.143	,	+0.182)
					, Point3f(	-0.255,		-0.143	,	+0.006	)
					, Point3f(	-0.255,		-0.005	,	+0.006	)
					, Point3f(	-0.255,		-0.005	,	+0.182)});
	corners.push_back({Point3f(	-0.255,		-0.143	,	-0.006	)
					, Point3f(	-0.255,		-0.143	,	-0.182)
					, Point3f(	-0.255,		-0.005	,	-0.182)
					, Point3f(	-0.255,		-0.005	,	-0.006	)});
	corners.push_back({Point3f(	-0.255,		-0.143	,	-0.193)
					, Point3f(	-0.255,		-0.143	,	-0.368	)
					, Point3f(	-0.255,		-0.005	,	-0.368)
					, Point3f(	-0.255,		-0.005	,	-0.193)});
	corners.push_back({Point3f(	-0.255,		+0.005	,	+0.368)// bottom row
					, Point3f(	-0.255,		+0.005	,	+0.193)
					, Point3f(	-0.255,		+0.143	,	+0.193)
					, Point3f(	-0.255,		+0.143	,	+0.368)});
	corners.push_back({Point3f(	-0.255,		+0.005	,	+0.182)
					, Point3f(	-0.255,		+0.005	,	+0.006)
					, Point3f(	-0.255,		+0.143	,	+0.006)
					, Point3f(	-0.255,		+0.143	,	+0.182)});
	corners.push_back({Point3f(	-0.255,		+0.005	,	-0.006	)
					, Point3f(	-0.255, 	+0.005	,	-0.182)
					, Point3f(	-0.255,		+0.143	,	-0.182)
					, Point3f(	-0.255,		+0.143	,	-0.006	)});
	corners.push_back({Point3f(	-0.255,		+0.005	,	-0.193)
					, Point3f(	-0.255,		+0.005	,	-0.368	)
					, Point3f(	-0.255,		+0.143	,	-0.368	)
					, Point3f(	-0.255,		+0.143	,	-0.193)});
#include "bottom_markers.cpp"
	assert(ids.size() == corners.size());	
	_markers = aruco::Board::create(InputArrayOfArrays(corners), dict, InputArray(ids));

	ids.clear();
	corners.clear();
	for (auto i=2*(6+8); i < 2*(6+8) + 12*16; ++i) { // bottom
		ids.push_back(i);
	}
#include "bottom_markers.cpp"
	assert(ids.size() == corners.size());	
	_btmMarkers = aruco::Board::create(InputArrayOfArrays(corners), dict, InputArray(ids));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "maruco");
	Maruco pub;
	ros::spin();

	return 0;
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

void Maruco::onQuadFrame(const sensor_msgs::ImageConstPtr& msg) {
	// ROS_INFO("onQuadFrame %s", msg->header.frame_id.c_str());
	int id = msg->header.frame_id[4] - '0';
	if (id > 3) {
		ROS_INFO("Image from unexpected camera frame %s", msg->header.frame_id.c_str());
		return;
	}

	if (_quadIntrinsic[id].at<double>(0,0) == 0) {
		ROS_INFO("quad camera %i not yet calibrated", id);
		return;
	}
	try {
		Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		aruco::detectMarkers(frame, _markers->dictionary, markerCorners, markerIds
			, _arucoDetectionParam, noArray() //rejectedImgPoints, 
			, _quadIntrinsic[id], _quadDistortion[id]
			);
		Vec3d rvec, tvec;
		if(markerIds.size() <= 0
			|| !aruco::estimatePoseBoard(markerCorners, markerIds, _markers
					, _quadIntrinsic[id], _quadDistortion[id] //, rvec, tvec
					, _quadState[id].rvec, _quadState[id].tvec
					// , cv::SOLVEPNP_P3P
					)) {
			return;
		}
		rvec = _quadState[id].rvec; tvec = _quadState[id].tvec;
		string markerIdStr = format("%d", markerIds[0]);
		for (auto j=1; j < markerIds.size(); ++j) {
			markerIdStr += format(",%d", markerIds[j]);
		}

		// output rotation vector is an angle * axis formulation
		// cvRodrigues2() converts rotation vector to to a 3-by-3 rotation matrix  

		// Record the latest valid board observation score
		float angle = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
		float sina2 = sin(0.5f * angle);
		float scale = sina2 / angle;

		ROS_DEBUG_THROTTLE(1,
				"markers (%s) in cam%u; T = [%.2f, %.2f, %.2f] R = [%.2f, %.2f, %.2f]"
				, markerIdStr.c_str(), id
				, tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2]);

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
		tf2::Quaternion Q(rvec[2] * scale, -rvec[0] * scale, -rvec[1] * scale
						, cos(0.5f * angle));

		geometry_msgs::PoseStamped cam2marker;
		cam2marker.pose.orientation.x = Q.x();
		cam2marker.pose.orientation.y = Q.y();
		cam2marker.pose.orientation.z = Q.z();
		cam2marker.pose.orientation.w = Q.w();
		cam2marker.pose.position.x =  tvec[2];
		cam2marker.pose.position.y = -tvec[0];
		cam2marker.pose.position.z = -tvec[1];
		cam2marker.header.stamp = msg->header.stamp;
		cam2marker.header.frame_id = format("quad%d_link", id);
		static uint32_t sSeq = 0;
		cam2marker.header.seq = ++sSeq;

		try {
		    geometry_msgs::PoseStamped quad2marker;
			tf2_buffer_.transform(cam2marker, quad2marker, "quad_link");
			if (quad2marker.header.stamp <= _marker2QuadTime) {
				// ignoring redundant observation
				return;
			}

			tf2::Quaternion Q;
			tf2::fromMsg(quad2marker.pose.orientation, Q);
			ROS_DEBUG_THROTTLE(1, //"%d.%03u "
				"cam %d [%.3f, %.3f; %.2f, %.2f, %.2f, %.2f] in quad_link"
				//, quad2marker.header.stamp.sec, quad2marker.header.stamp.nsec/1000000
				, id
				, quad2marker.pose.position.x, quad2marker.pose.position.y
				, quad2marker.pose.orientation.x
				, quad2marker.pose.orientation.y
				, quad2marker.pose.orientation.z
				, quad2marker.pose.orientation.w
			);

			auto T = quad2marker.pose.position;

			if (_prevQuadPosition[0]) {
#if 1
				auto dx = T.x - _prevQuadPosition[0], dy = T.y - _prevQuadPosition[1];
				if (dx*dx + dy*dy > (0.3*0.3) * _wheel_base * _wheel_base
					&& msg->header.stamp - _marker2QuadTime < _markObsDeadlne) {
					ROS_WARN_THROTTLE(1, "Quad estimate outlier; dropping estimate");
					return;
				}
#endif
#if 1
				if (msg->header.stamp - _marker2MonoTime <= _markObsDeadlne) {
					ROS_INFO_THROTTLE(2,
						"mono estimate is current; skipping backup localization estimate");
					return;
				}
#endif
			}
			_marker2QuadTime = quad2marker.header.stamp;
			_prevQuadPosition[0] = T.x; _prevQuadPosition[1] = T.y;

			geometry_msgs::TransformStamped xf;
			xf.header = quad2marker.header;
			// Assume the vehicle ONLY yaws
			double yaw = 0;
			tf2::Vector3 axis = Q.getAxis();
			if (abs(axis[2]) > 0.8) { // rotation axis roughly along vertical
				// => can assume that the rotation amount is the yaw
				yaw = Q.getAngle() * (1 - 2*signbit(axis[2]));
			}

			xf.child_frame_id = "trailer"; // aruco and trailer are coincident
			xf.transform.translation.x = T.x;
			xf.transform.translation.y = T.y;
			xf.transform.translation.z = T.z;
			xf.transform.rotation.x = 0; //Qave.x();
			xf.transform.rotation.y = 0; //Qave.y();
			xf.transform.rotation.z = sin(0.5 * yaw); //Qave.z();
			xf.transform.rotation.w = cos(0.5 * yaw); //Qave.w();
			_br.sendTransform(xf);

			ROS_DEBUG(//"%d.%03u "
				"trailer <-- quad_link = [%.2f, %.2f; (%.2f, %.2f, %.2f), %.2f]"
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
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("quad2 marker transform failure %s\n", ex.what());
		}

		if (_show_axis) { // Show the board frame
		  	cv::aruco::drawAxis(frame, _quadIntrinsic[id], _quadDistortion[id], rvec, tvec, 0.8);
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			// preserve the timestamp from the image frame
			img->header.stamp = msg->header.stamp;
			_debugImgPub.publish(img);
		}
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Cannot get quad image");
	}
}

void Maruco::onMonoFrame(const sensor_msgs::ImageConstPtr& msg) {
	if (_monoIntrinsic.at<double>(0,0) == 0) {
		ROS_INFO("webcam not yet calibrated");
		return;
	}
	try {
		Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		aruco::detectMarkers(frame, _markers->dictionary, markerCorners, markerIds
			, _arucoDetectionParam, noArray() //rejectedImgPoints, 
			, _monoIntrinsic, _monoDistortion);
		Vec3d rvec, tvec;
		if(markerIds.size() <= 0
			|| !aruco::estimatePoseBoard(markerCorners, markerIds, _markers
					, _monoIntrinsic, _monoDistortion //, rvec, tvec
					, _monoState.rvec, _monoState.tvec
					// , cv::SOLVEPNP_P3P// yield wrong answer
					)) {
			return;
		}

		rvec[0] = pify(_monoState.rvec[0]);
		rvec[1] = pify(_monoState.rvec[1]);
		rvec[2] = pify(_monoState.rvec[2]);
		tvec = _monoState.tvec;
		_monoState.rvec = rvec;
#if 0
		if (_monoState.tvec[2]) {
			const auto diff = tvec - _monoState.tvec
					, diff2 = diff * diff;
			if (diff2[0] + diff2[1] + diff2[2] > (0.1*0.1) * _wheel_base * _wheel_base) {
				ROS_ERROR("Insane aruco pose jump; dropping");
				return;
			}
		}
#endif

		string markerIdStr = format("%d", markerIds[0]);
		for (auto i=1; i < markerIds.size(); ++i) {
			markerIdStr += format(",%d", markerIds[i]);
		}

		float angle = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
		float sina2 = sin(0.5f * angle);
		float scale = sina2 / angle;

		ROS_DEBUG_THROTTLE(1,
				"markers (%s) in webcam; T = [%.2f, %.2f, %.2f] R = [%.2f, %.2f, %.2f]"
				, markerIdStr.c_str()
				, tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2]);
		tf2::Quaternion Q(rvec[2] * scale, -rvec[0] * scale, -rvec[1] * scale
						, cos(0.5f * angle));
#if 0
		if (msg->header.stamp - _marker2QuadTime <= _markObsDeadlne) {
			ROS_DEBUG_THROTTLE(1,
				"quad estimate is current; skipping backup localization estimate");
			return;
		}
#endif
		// if (Q.w() < 0) { Q.x = -Q.x(); } 

		geometry_msgs::PoseStamped cam2marker;
		cam2marker.pose.orientation.x = Q.x();
		cam2marker.pose.orientation.y = Q.y();
		cam2marker.pose.orientation.z = Q.z();
		cam2marker.pose.orientation.w = Q.w();
		cam2marker.pose.position.x =  tvec[2];
		cam2marker.pose.position.y = -tvec[0];
		cam2marker.pose.position.z = -tvec[1];
		cam2marker.header.stamp = msg->header.stamp;
		cam2marker.header.frame_id = "webcam_link";
		static uint32_t sSeq = 0;
		cam2marker.header.seq = ++sSeq;
		try {
		    geometry_msgs::PoseStamped base2marker;
			tf2_buffer_.transform(cam2marker, base2marker, "base_link");
			tf2::Quaternion Q;
			tf2::fromMsg(base2marker.pose.orientation, Q);
			ROS_DEBUG_THROTTLE(1, //"%d.%03u "
				"trailer [%.3f, %.3f; %.2f, %.2f, %.2f, %.2f] in base_link (webcam)"
				//, base2marker.header.stamp.sec, base2marker.header.stamp.nsec/1000000
				, base2marker.pose.position.x, base2marker.pose.position.y
				, base2marker.pose.orientation.x
				, base2marker.pose.orientation.y
				, base2marker.pose.orientation.z
				, base2marker.pose.orientation.w
			);

			auto T = base2marker.pose.position;

			if (_prevMonoPosition[0]) {
#if 1
				auto dx = T.x - _prevMonoPosition[0], dy = T.y - _prevMonoPosition[1];
				if (dx*dx + dy*dy > (0.3*0.3)*_wheel_base * _wheel_base) {
					ROS_ERROR("mono estimate outlier delta = (%.2f, %.2f)"
							, dx, dy);
					return;
				}
#endif
#if 0
				if (msg->header.stamp - _marker2QuadTime < _markObsDeadlne) {
					ROS_WARN("ignoring mono in favor of recent quad");
					return;
				}
#endif
			}
			_marker2MonoTime = cam2marker.header.stamp;
			_prevMonoPosition[0] = T.x; _prevMonoPosition[1] = T.y;

			geometry_msgs::TransformStamped xf;
			xf.header = base2marker.header;
			// Assume the vehicle ONLY yaws
			double yaw = 0;
			tf2::Vector3 axis = Q.getAxis();
			if (abs(axis[2]) > 0.8) { // rotation axis roughly along vertical
				// => can assume that the rotation amount is the yaw
				yaw = Q.getAngle() * (1 - 2*signbit(axis[2]));
			}

			xf.child_frame_id = "trailer"; // aruco and trailer are coincident
			xf.transform.translation.x = T.x;
			xf.transform.translation.y = T.y;
			xf.transform.translation.z = T.z;
			xf.transform.rotation.x = 0; //Qave.x();
			xf.transform.rotation.y = 0; //Qave.y();
			xf.transform.rotation.z = sin(0.5 * yaw); //Qave.z();
			xf.transform.rotation.w = cos(0.5 * yaw); //Qave.w();
			_br.sendTransform(xf);

			ROS_DEBUG(//"%d.%03u "
				"trailer <-- base_link = [%.2f, %.2f; (%.2f, %.2f, %.2f), %.2f]"
				//, xf.header.stamp.sec, xf.header.stamp.nsec/1000000
				, xf.transform.translation.x, xf.transform.translation.y
				, axis[0], axis[1], axis[2], yaw);
			// tell tf2 listeners that there is a new tf2 update
			aruco_tf_strobe_.publish(xf.header);
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("base_lihk to marker transform failure %s\n", ex.what());
		}

		if (_show_axis) { // Show the board frame
		  	cv::aruco::drawAxis(frame, _monoIntrinsic, _monoDistortion, rvec, tvec, 0.8);
			auto img = boost::make_shared<sensor_msgs::Image>();
			convert_frame_to_message(frame, img);
			// preserve the timestamp from the image frame
			img->header.stamp = msg->header.stamp;
			_debugImgPub.publish(img);
		}
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Cannot get webcam image");
	}
}