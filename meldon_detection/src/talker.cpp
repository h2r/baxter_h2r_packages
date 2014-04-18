#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include "libkerneldesc.cc"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <meldon_detection/RecognizedObjectArray.h>
#include <cv.h>
static unsigned int LOC_MODEL_TYPE=3; //0 or 3
const char* model_name = model_names[LOC_MODEL_TYPE];
const char* model_file = model_files[LOC_MODEL_TYPE];
const char* model_var = model_vars[LOC_MODEL_TYPE];
const char* param_file = param_files[LOC_MODEL_TYPE];
KernelDescManager* kdm;
vector<cv::Point> top;
vector<cv::Point> bottom;
cv::Mat cam_img;
ros::Publisher rec_objs;
bool real_img = false;
bool vis = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv::Rect bound;
	cv::Mat boxed;
	try{
		if(LOC_MODEL_TYPE == 0){
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}else{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		}
		cam_img = cv_ptr->image;
		real_img = true;
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if(vis){
		for(int i =0; i<top.size(); i++){
			rectangle(cv_ptr->image, top[i], bottom[i], cv::Scalar(0,0,255));
		}
		cv::imshow("Object Viewer", cv_ptr->image);
		cv::waitKey(1);
	}
}

void clusterCallback(const visualization_msgs::MarkerArray& msg){
	if(real_img){
		meldon_detection::RecognizedObjectArray to_send;
		to_send.objects.resize(msg.markers.size());
		MatrixXf imfea2;
		VectorXf scores;
		IplImage temp = cam_img;
		int height = cam_img.size().height;
		int width = cam_img.size().width;
		ROS_INFO("(%d,%d)", height, width);
		float minx, miny, maxx, maxy, px, py;
		cv::Rect bound;
		cv::Mat boxed;
		geometry_msgs::Point p;
		top.resize(msg.markers.size());
		bottom.resize(msg.markers.size());
		ROS_INFO("Objects found: %d", msg.markers.size());
		float f;
		std::string res;
		if(LOC_MODEL_TYPE == 0) f= 580;
		else f=525;
		float cx = (width/2) - 0.5;
		float cy = (height/2) - 0.5;
		for(int i=0; i<msg.markers.size(); i++){
			minx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			maxx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			miny = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			maxy = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			for(int j = 0; j < msg.markers[i].points.size(); j++){
				p = msg.markers[i].points[j];
				px = f*(p.x/p.z) + cx;
				py = f*(p.y/p.z) + cy;
				if(px < minx){minx = px;}
				if(px > maxx){maxx = px;}
				if(py < miny){miny = py;}
				if(py > maxy){maxy = py;}
			}
			top[i] = cv::Point(minx - 5, miny - 5);
			bottom[i] = cv::Point(maxx + 5, maxy + 5);
			if(top[i].x < 0){top[i].x = 0;}
			else if(top[i].x > width - 1){top[i].x = width-1;}
			if(top[i].y < 0) top[i].y = 0;
			else if(top[i].y > height-1){top[i].y = height-1;}
			if(bottom[i].x < 0){bottom[i].x = 0;}
			else if(bottom[i].x > width - 1){bottom[i].x = width-1;}
			if(bottom[i].y < 0) bottom[i].y = 0;
			else if(bottom[i].y > height-1){bottom[i].y = height-1;}
		}/*
		for(int i=0; i<top.size(); i++){
			ROS_INFO("BOXED: (%d, %d) to (%d, %d)", top[i].x, top[i].y, bottom[i].x, bottom[i].y);
			bound = cv::Rect(top[i].x, top[i].y, bottom[i].x - top[i].x, bottom[i].y - top[i].y);
			boxed = cam_img(bound);
			temp = boxed;
			kdm->Process(imfea2, &temp);
			kdm->Classify(scores, imfea2);
			res = kdm->GetObjectName(scores);
			ROS_INFO("Tabletop %d identified as %s", i, res.c_str());
			meldon_detection::RecognizedObject to_add;
			to_add.points = msg.markers[i].points;
			to_add.name = res.c_str();
			to_send.objects[i] = to_add;
		}*/
		temp = cam_img;
		kdm->Process(imfea2, &temp);
		kdm->Classify(scores, imfea2);
		rec_objs.publish(to_send);
	}
	ROS_INFO("Identification complete");
}
int main(int argc, char **argv){
	ros::init(argc, argv, "meldon_detection");
	if(argc < 2){
		ROS_ERROR("Please provide the location the meldon_detection package so that the model files can be located");
		exit(0);
	}
	ros::NodeHandle n;
	std::string s;
	image_transport::Subscriber image_sub;
	kdm = new KernelDescManager(argv[1] + string("/src/kdes/KernelDescriptors_CPU"),string(model_name), string(model_file), string(model_var), string(param_file), LOC_MODEL_TYPE, MAX_IMAGE_SIZE);
	image_transport::ImageTransport it(n);
	ros::Subscriber clusters = n.subscribe("/tabletop/clusters", 1, clusterCallback);
	rec_objs = n.advertise<meldon_detection::RecognizedObjectArray>("labeled_objects", 10);
	if(LOC_MODEL_TYPE == 0){
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
	}else{
		image_sub = it.subscribe("/camera/depth_registered/image_raw", 1, imageCallback);
	}
	cv::namedWindow("Object Viewer");
	ros::spin();
	return 0;
}