#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <baxter_kinect_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    tf::Matrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2));
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();
  tf::Vector3 origin = trans.getOrigin();
  
  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());
  
  out.topLeftCorner<3,3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3,1>() = origin_out;
  out(3,3) = 1;
  
  return out;
}


class CalibrateKinectCheckerboard
{
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher detector_pub_;
    ros::Publisher physical_pub_;
  
    // Image and camera info subscribers;
    ros::Subscriber kinectImage_sub_; 
    ros::Subscriber armImage_sub_; 
    ros::Subscriber kinectInfo_sub_;
    ros::Subscriber armInfo_sub_;
    
    // Structures for interacting with ROS messages
    cv_bridge::CvImagePtr input_bridge_;
    cv_bridge::CvImagePtr output_bridge_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    image_geometry::PinholeCameraModel kinectCamModel_;
    image_geometry::PinholeCameraModel armCamModel_;
    
    // Calibration objects
    PatternDetector kinectPatternDetector_;
    PatternDetector armPatternDetector_;
    // The optimized transform
    Eigen::Transform<float, 3, Eigen::Affine> transform_;
    
    // Visualization for markers
    pcl::PointCloud<pcl::PointXYZ> detector_points_;
    //pcl::PointCloud<pcl::PointXYZ> ideal_points_;
    pcl::PointCloud<pcl::PointXYZ> image_points_;
    pcl::PointCloud<pcl::PointXYZ> physical_points_;
    
    // Have we calibrated the camera yet?
    bool calibrated;
    
    ros::Timer timer_;
    
    // Parameters
    std::string fixed_frame;
    std::string kinect_frame;
    std::string arm_camera_frame;
    std::string target_frame;
    std::string tip_frame;
    std::string touch_frame;
    
    int checkerboard_width;
    int checkerboard_height;
    double checkerboard_grid;
     
    tf::Transform kinectTransform;
    tf::Transform armCameraTransform;
    std::string kinectFrameID;
    std::string armCameraFrameID;

    bool haveKinectInfo;
    bool haveArmCameraInfo;
    bool haveArmImage;
    // Gripper tip position
    //geometry_msgs::PointStamped gripper_tip;

public:
  CalibrateKinectCheckerboard()
    : nh_("~"), it_(nh_), calibrated(false)
  {
    // Load parameters from the server.
    nh_.param<std::string>("fixed_frame", fixed_frame, "/world");
    nh_.param<std::string>("kinect_frame", kinect_frame, "/camera_rgb_optical_frame");
    nh_.param<std::string>("arm_camera_frame", arm_camera_frame, "/left_hand_camera");
    nh_.param<std::string>("target_frame", target_frame, "/calibration_pattern");
    nh_.param<std::string>("tip_frame", tip_frame, "/left_gripper");
    
    nh_.param<int>("checkerboard_width", checkerboard_width, 5);
    nh_.param<int>("checkerboard_height", checkerboard_height, 4);
    nh_.param<double>("checkerboard_grid", checkerboard_grid, 0.0245);
    
    // Set pattern detector sizes
    kinectPatternDetector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    armPatternDetector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    transform_.translation().setZero();
    transform_.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();
    
    // Create subscriptions
    kinectInfo_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &CalibrateKinectCheckerboard::kinectInfoCallback, this);
    armInfo_sub_ = nh_.subscribe("/cameras/left_hand_camera/camera_info", 1, &CalibrateKinectCheckerboard::armInfoCallback, this);
    
    // Also publishers
    pub_ = it_.advertise("calibration_pattern_out", 1);
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    physical_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("physical_points_cloud", 1);
    
    // Create ideal points
    //ideal_points_.push_back( pcl::PointXYZ(0, 0, 0) );
    //ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, 0, 0) );
    //ideal_points_.push_back( pcl::PointXYZ(0, (checkerboard_height-1)*checkerboard_grid, 0) );
    //ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, (checkerboard_height-1)*checkerboard_grid, 0) );
    
    // Create proper gripper tip point
    //nh_.param<double>("gripper_tip_x", gripper_tip.point.x, 0.0);
    //nh_.param<double>("gripper_tip_y", gripper_tip.point.y, 0.0);
    //nh_.param<double>("gripper_tip_z", gripper_tip.point.z, 0.0);
    //gripper_tip.header.frame_id = tip_frame;

    this->haveArmImage = false;
    this->haveKinectInfo = false;
    this->haveArmCameraInfo = false;
    ROS_INFO("[calibrate] Initialized.");
  }

  void kinectInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (this->haveKinectInfo) 
    {
      return;
    }
    kinectCamModel_.fromCameraInfo(info_msg);
    kinectPatternDetector_.setCameraMatrices(kinectCamModel_.intrinsicMatrix(), kinectCamModel_.distortionCoeffs());
    kinectImage_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &CalibrateKinectCheckerboard::kinectImageCallback, this);
    this->haveKinectInfo = true;
    ROS_INFO("[calibrate] Got kinect image info!");
  }

  void armInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (this->haveArmCameraInfo)
    {
      return;
    }
    armCamModel_.fromCameraInfo(info_msg);
    armPatternDetector_.setCameraMatrices(armCamModel_.intrinsicMatrix(), armCamModel_.distortionCoeffs());
    armImage_sub_ = nh_.subscribe("/cameras/left_hand_camera/image", 1, &CalibrateKinectCheckerboard::armImageCallback, this);
    this->haveArmCameraInfo = true;
    ROS_INFO("[calibrate] Got arm image info!");
  }
  
  void kinectImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    try
    {
      input_bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
      output_bridge_ = cv_bridge::toCvCopy(image_msg, "bgr8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }
  
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    if (!kinectPatternDetector_.detectPattern(input_bridge_->image, translation, orientation, output_bridge_->image))
    {
      ROS_INFO("[calibrate] Couldn't detect checkerboard in kinect camera, make sure it's visible in the image.");
      return;
    }
    

    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
                                   
      kinectTransform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
       kinectTransform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );
       tf_broadcaster_.sendTransform(tf::StampedTransform(kinectTransform, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    this->kinectFrameID = image_msg->header.frame_id;
    if (this->haveArmImage)
    {
       calibrate();

    }
  }
  
  void armImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    try
    {
      input_bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
      output_bridge_ = cv_bridge::toCvCopy(image_msg, "bgr8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }
  
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    if (!armPatternDetector_.detectPattern(input_bridge_->image, translation, orientation, output_bridge_->image))
    {
      ROS_INFO("[calibrate] Couldn't detect checkerboard in arm camera, make sure it's visible in the image.");
      return;
    }
    
    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
                                   
      armCameraTransform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
      armCameraTransform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );
      tf_broadcaster_.sendTransform(tf::StampedTransform(armCameraTransform, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    this->armCameraFrameID = image_msg->header.frame_id;
    this->haveArmImage = true;
    //publishCloud(ideal_points_, target_transform, image_msg->header.frame_id);
    
    //overlayPoints(ideal_points_, target_transform, output_bridge_);
    
    // Publish calibration image
    //pub_.publish(output_bridge_->toImageMsg());
    
    //pcl_ros::transformPointCloud(ideal_points_, image_points_, target_transform);
    
    //cout << "Got an image callback!" << endl;
    
    //calibrate(image_msg->header.frame_id);
    
    //ros::shutdown();
  }
  
  
  bool calibrate()
  {
    
    Eigen::Matrix4f t;
    
    // Output       
    tf::Transform transform = tfFromEigen(t), trans_full, arm_transform_unstamped, kinect_transform_unstamped, arm_camera_transform_unstamped;
    
  
    
    tf::StampedTransform arm_transform;

    try
    {
      tf_listener_.lookupTransform(arm_camera_frame, fixed_frame, ros::Time(0), arm_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }
    //cout << "Resulting transform (arm camera frame -> world frame): " << endl << arm_transform << endl << endl;
    /*
    tf::StampedTransform arm_camera_transform;
    try
    {
      tf_listener_.lookupTransform(arm_camera_frame, target_frame, ros::Time(0), arm_camera_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }
    cout << "Resulting transform (arm camera frame -> target frame): " << endl << arm_camera_transform << endl << endl;
    
    tf::StampedTransform kinect_transform;
    try
    {
      tf_listener_.lookupTransform(target_frame, kinect_frame, ros::Time(0), kinect_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }
    cout << "Resulting transform (target frame -> kinect frame): " << endl << kinect_transform << endl << endl;
    */
    arm_transform_unstamped = arm_transform;
    trans_full = arm_transform_unstamped * this->armCameraTransform * this->kinectTransform.inverse();
    
    Eigen::Matrix4f t_full = EigenFromTF(trans_full);
    Eigen::Matrix4f t_full_inv = (Eigen::Transform<float,3,Affine>(t_full).inverse()).matrix();
    
    cout << "Resulting transform (world frame -> kinect frame): " << endl << t_full << endl << endl;
    printStaticTransform(t_full_inv, fixed_frame, kinect_frame);

    return true;
  }
  
  void printStaticTransform(Eigen::Matrix4f& transform, const std::string frame1, const std::string frame2)
  {
    Eigen::Quaternionf quat(transform.topLeftCorner<3,3>() );
    Eigen::Vector3f translation(transform.topRightCorner<3,1>() );
    
    cout << "Static transform publisher (use for external kinect): " << endl;
    
    cout << "rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms" << endl;
    cout << "rosrun tf static_transform_publisher " << translation.x() << " "
         << translation.y() << " " << translation.z() << " " 
         << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
         << frame1 << " " << frame2 << " 100" << endl << endl;
         
    tf::Transform temp_tf_trans = tfFromEigen(transform);
    
    double yaw, pitch, roll;
    
    std::string fixed_frame_urdf(fixed_frame);
    
    // If there's a leading '/' character, remove it, as xacro can't deal with 
    // extra characters in the link name.
    if (fixed_frame_urdf.size() > 0 && fixed_frame_urdf[0] == '/')
      fixed_frame_urdf.erase(0, 1);
    
    temp_tf_trans.getBasis().getEulerYPR(yaw, pitch, roll);
    
    cout << "URDF output (use for kinect on robot): " << endl;
    
    cout << "<?xml version=\"1.0\"?>\n<robot>\n" << 
          "\t<property name=\"turtlebot_calib_cam_x\" value=\"" << translation.x() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_y\" value=\"" << translation.y() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_z\" value=\"" << translation.z() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_rr\" value=\"" << roll << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_rp\" value=\"" << pitch << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_ry\" value=\"" << yaw << "\" />\n" <<
          "\t<property name=\"turtlebot_kinect_frame_name\" value=\"" << fixed_frame_urdf << "\" />\n" <<
          "</robot>" << endl << endl;
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_kinect_arm");
  
  CalibrateKinectCheckerboard cal;
  ros::spin();
}
