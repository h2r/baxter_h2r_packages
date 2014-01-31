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
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>

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

    // Marker topic subscriber
    ros::Subscriber alvarMarkersSubscriber;
   
    // Structures for interacting with ROS messages
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // The optimized transform
    Eigen::Transform<float, 3, Eigen::Affine> transform_;

    std::set<std::string> observedTFFrames;
    std::set<int> observedMarkers;
    std::string uncalibratedFrame;
    std::string baseFrame;
    
    // Have we calibrated the camera yet?
    bool calibrated;
    
    // Parameters
    std::string fixed_frame;
    std::string kinect_frame;
    std::string arm_camera_frame;
    std::string target_frame;
    std::string tip_frame;
    std::string touch_frame;
    
    tf::Transform kinectTransform;
    tf::Transform armCameraTransform;
    std::string kinectFrameID;
    std::string armCameraFrameID;

public:
  CalibrateKinectCheckerboard()
    : nh_("~"), calibrated(false)
  {
    // Load parameters from the server.
    nh_.param<std::string>("fixed_frame", fixed_frame, "/world");
    nh_.param<std::string>("kinect_frame", kinect_frame, "/camera_rgb_optical_frame");
    nh_.param<std::string>("arm_camera_frame", arm_camera_frame, "/left_hand_camera");
    nh_.param<std::string>("target_frame", target_frame, "/calibration_pattern");
    nh_.param<std::string>("tip_frame", tip_frame, "/left_gripper");
    this->uncalibratedFrame = "/camera_rgb_optical_frame";    
    this->baseFrame = "/world";

    // Create subscriptions
    ROS_INFO("Subscribing to /ar_pose_marker");
    alvarMarkersSubscriber = nh_.subscribe("/ar_pose_marker", 1, &CalibrateKinectCheckerboard::alvarMarkersCallback, this);
    ROS_INFO("[calibrate] Initialized.");
  }

  void alvarMarkersCallback(const ar_track_alvar::AlvarMarkers markersMsg)
  {
    for (int i = 0; i < markersMsg.markers.size(); ++i)
    {
        this->observedMarkers.insert(markersMsg.markers[i].id);
    }

    for (std::set<int>::iterator it=this->observedMarkers.begin(); it!=this->observedMarkers.end(); ++it)
    {
           std::stringstream arMarkerStream;
           arMarkerStream << "ar_marker_" << *it;

           tf::StampedTransform uncalibratedInverseTransform, calibratedTransform;
           tf::Transform uncalibratedTransform, fullTransform;
           try
           {
              tf_listener_.lookupTransform(arMarkerStream.str(), this->uncalibratedFrame , ros::Time(0), uncalibratedInverseTransform);
           }
           catch (tf::TransformException& ex)
           {
             ROS_WARN_STREAM("Marker " << *it << " does not have a transform from frame id " << this->uncalibratedFrame);
           }
           try
           {
              tf_listener_.lookupTransform(this->baseFrame, arMarkerStream.str() , ros::Time(0), calibratedTransform);
           }
           catch (tf::TransformException& ex)
           {
             ROS_WARN_STREAM("Marker " << *it << " does not have a transform from frame id " << this->baseFrame);
           }
           
           fullTransform = calibratedTransform * uncalibratedTransform;
           tf_broadcaster_.sendTransform(tf::StampedTransform(fullTransform, ros::Time::now(), this->baseFrame, this->uncalibratedFrame));
           //Eigen::Matrix4f matTransform = EigenFromTF(fullTransform);
           // this->printStaticTransform(matTransform, fixed_frame, kinect_frame);
     } 
    
     
     

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
