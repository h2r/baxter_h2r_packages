#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar/ParamsConfig.h>

#include <deque>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>

#include <tf/tf.h>
#include <Eigen/Core>
#include <ar_track_alvar/filter/kinect_filtering.h>
#include <ar_track_alvar/filter/medianFilter.h>
#include <fstream>
#include <iostream>

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

namespace gm=geometry_msgs;
namespace ata=ar_track_alvar;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

using namespace alvar;
using namespace std;
using boost::make_shared;

Camera *kinectCam;
Camera *calibratedCam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber kinectSub_;
image_transport::Subscriber calibratedSub_;
ros::Subscriber cloud_sub_;
ros::Publisher rvizMarkerPub_;
ros::Publisher rvizMarkerPub2_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;
MarkerDetector<MarkerData> kinect_marker_detector;
MultiMarkerBundle **multi_marker_bundles=NULL;

Pose *bundlePoses;
Pose *kinectBundlePoses;
int *master_id;
int *bundles_seen;
bool *master_visible;
std::vector<int> *bundle_indices;   
bool init = true;  

ata::MedianFilter **med_filts;
int med_filt_size;

std::string launchFileName;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string kinectImageTopic; 
std::string kinectInfoTopic; 
std::string kinectBaseLinkFrameID;
std::string output_frame;

std::string calibratedImageTopic; 
std::string calibratedInfoTopic; 
std::string calibratedFrameID;
int n_bundles = 0;   

std::deque<tf::Transform> kinectTransforms;
tf::Transform kinectTransform;
std::deque<std::deque<tf::Transform> > camTransforms;
std::vector<tf::Transform> camTransform;


//Debugging utility function
void draw3dPoints(ARCloud::Ptr cloud, string frame, int color, int id, double rad)
{
  visualization_msgs::Marker rvizMarker;

  rvizMarker.header.frame_id = frame;
  rvizMarker.header.stamp = ros::Time::now(); 
  rvizMarker.id = id;
  rvizMarker.ns = "3dpts";
  
  rvizMarker.scale.x = rad;
  rvizMarker.scale.y = rad;
  rvizMarker.scale.z = rad;
  
  rvizMarker.type = visualization_msgs::Marker::SPHERE_LIST;
  rvizMarker.action = visualization_msgs::Marker::ADD;
  
  if(color==1){
    rvizMarker.color.r = 0.0f;
    rvizMarker.color.g = 1.0f;
    rvizMarker.color.b = 1.0f;
    rvizMarker.color.a = 1.0;
  }
  if(color==2){
    rvizMarker.color.r = 1.0f;
    rvizMarker.color.g = 0.0f;
    rvizMarker.color.b = 1.0f;
    rvizMarker.color.a = 1.0;
  }
  if(color==3){
    rvizMarker.color.r = 1.0f;
    rvizMarker.color.g = 1.0f;
    rvizMarker.color.b = 0.0f;
    rvizMarker.color.a = 1.0;
  }
  
  gm::Point p;
  for(int i=0; i<cloud->points.size(); i++){
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    rvizMarker.points.push_back(p);
  }
  
  rvizMarker.lifetime = ros::Duration (1.0);
  rvizMarkerPub2_.publish (rvizMarker);
}


void drawArrow(gm::Point start, tf::Matrix3x3 mat, string frame, int color, int id)
{
  visualization_msgs::Marker rvizMarker;
  
  rvizMarker.header.frame_id = frame;
  rvizMarker.header.stamp = ros::Time::now(); 
  rvizMarker.id = id;
  rvizMarker.ns = "arrow";
  
  rvizMarker.scale.x = 0.01;
  rvizMarker.scale.y = 0.01;
  rvizMarker.scale.z = 0.1;
  
  rvizMarker.type = visualization_msgs::Marker::ARROW;
  rvizMarker.action = visualization_msgs::Marker::ADD;
  
  for(int i=0; i<3; i++){
    rvizMarker.points.clear();  
    rvizMarker.points.push_back(start);
    gm::Point end;
    end.x = start.x + mat[0][i];
    end.y = start.y + mat[1][i];
    end.z = start.z + mat[2][i];
    rvizMarker.points.push_back(end);
    rvizMarker.id += 10*i;
    rvizMarker.lifetime = ros::Duration (1.0);

    if(color==1){
      rvizMarker.color.r = 1.0f;
      rvizMarker.color.g = 0.0f;
      rvizMarker.color.b = 0.0f;
      rvizMarker.color.a = 1.0;
    }
    if(color==2){
      rvizMarker.color.r = 0.0f;
      rvizMarker.color.g = 1.0f;
      rvizMarker.color.b = 0.0f;
      rvizMarker.color.a = 1.0;
    }
    if(color==3){
      rvizMarker.color.r = 0.0f;
      rvizMarker.color.g = 0.0f;
      rvizMarker.color.b = 1.0f;
      rvizMarker.color.a = 1.0;
    }
    color += 1;

    rvizMarkerPub2_.publish (rvizMarker);
  }
}


// Infer the master tag corner positons from the other observed tags
// Also does some of the bookkeeping for tracking that MultiMarker::_GetPose does 
int InferCorners(const ARCloud &cloud, MultiMarkerBundle &master, ARCloud &bund_corners){
  bund_corners.clear();
  bund_corners.resize(4);
  for(int i=0; i<4; i++){
    bund_corners[i].x = 0;
    bund_corners[i].y = 0;
    bund_corners[i].z = 0;
  }

  // Reset the marker_status to 1 for all markers in point_cloud for tracking purposes
  for (size_t i=0; i<master.marker_status.size(); i++) {
    if (master.marker_status[i] > 0) master.marker_status[i]=1;
  }

  int n_est = 0;

  // For every detected marker
  for (size_t i=0; i<kinect_marker_detector.markers->size(); i++)
    {
      const Marker* marker = &((*kinect_marker_detector.markers)[i]);
      int id = marker->GetId();
      int index = master.get_id_index(id);
      int mast_id = master.master_id;
      if (index < 0) continue;

      // But only if we have corresponding points in the pointcloud
      if (master.marker_status[index] > 0 && marker->valid) {
        n_est++;

        std::string marker_frame = "ar_marker_";
        std::stringstream mark_out;
        mark_out << id;
        std::string id_string = mark_out.str();
        marker_frame += id_string;

        //Grab the precomputed corner coords and correct for the weird Alvar coord system
        for(int j = 0; j < 4; ++j)
        {
            tf::Vector3 corner_coord = master.rel_corners[index][j];
            gm::PointStamped p, output_p;
            p.header.frame_id = marker_frame;
            p.point.x = corner_coord.y()/100.0;  
            p.point.y = -corner_coord.x()/100.0;
            p.point.z = corner_coord.z()/100.0;
            
            try{
                tf_listener->waitForTransform(cloud.header.frame_id, marker_frame, ros::Time(0), ros::Duration(0.1));
                tf_listener->transformPoint(cloud.header.frame_id, p, output_p);      
            }
            catch (tf::TransformException ex){
                ROS_ERROR("ERROR InferCorners: %s",ex.what());
                return -1;
            }

            bund_corners[j].x += output_p.point.x;
            bund_corners[j].y += output_p.point.y;
            bund_corners[j].z += output_p.point.z;
        }
        master.marker_status[index] = 2; // Used for tracking
      }
    }
  
  //Divide to take the average of the summed estimates
  if(n_est > 0){
    for(int i=0; i<4; i++){
        bund_corners[i].x /= n_est;
        bund_corners[i].y /= n_est;
        bund_corners[i].z /= n_est;
    }
  }

  return 0;
}

bool isCornerNaN(const ARPoint &point)
{
  return isnan(point.x) || isnan(point.y) || isnan(point.z);
}

int *getTwoValidPoints(const ARCloud &corners_3D, int *dir1, int *dir2)
{
  if(isCornerNaN(corners_3D[dir1[0]]) || isCornerNaN(corners_3D[dir1[1]]))
  {
    if(isCornerNaN(corners_3D[dir2[0]]) || isCornerNaN(corners_3D[dir2[1]]))
    {
      return NULL;
    }
    else{
      return dir2;
    } 
  }
  else{
    return dir1;
  }
}


int PlaneFitPoseImprovement(int id, const ARCloud &corners_3D, ARCloud::Ptr selected_points, const ARCloud &cloud, Pose &p){

  ata::PlaneFitResult res = ata::fitPlane(selected_points);
  gm::PoseStamped pose;
  pose.header.stamp = pcl_conversions::fromPCL(cloud.header).stamp;
  pose.header.frame_id = cloud.header.frame_id;
  pose.pose.position = ata::centroid(*res.inliers);
 
  //Get 2 points that point forward in marker x direction   
  int *xPoints = getTwoValidPoints(corners_3D, new int[2]{0,3}, new int[2]{1,2});
  int *yPoints = getTwoValidPoints(corners_3D, new int[2]{1,0}, new int[2]{2,3});  
  
  if (!xPoints || !yPoints)
  {
    return -1;
  }
  int i1 = xPoints[0], i2 = xPoints[1];
  int i3 = yPoints[0], i4 = yPoints[1];

  int succ = 
    ata::extractOrientation(res.coeffs, corners_3D[i1], corners_3D[i2], corners_3D[i3], corners_3D[i4], pose.pose.orientation);
  if(succ < 0) return -1;

  tf::Matrix3x3 mat;
  succ = ata::extractFrame(res.coeffs, corners_3D[i1], corners_3D[i2], corners_3D[i3], corners_3D[i4], mat);
  if(succ < 0) return -1;

  p.translation[0] = pose.pose.position.x * 100.0;
  p.translation[1] = pose.pose.position.y * 100.0;
  p.translation[2] = pose.pose.position.z * 100.0;
  p.quaternion[1] = pose.pose.orientation.x;
  p.quaternion[2] = pose.pose.orientation.y;
  p.quaternion[3] = pose.pose.orientation.z;
  p.quaternion[0] = pose.pose.orientation.w; 

  return 0;
}

// Updates the bundlePoses of the multi_marker_bundles by detecting markers and using all markers in a bundle to infer the master tag's position
void GetMultiMarkerPoses(IplImage *image) {

  if (marker_detector.Detect(image, calibratedCam, true, false, max_new_marker_error, max_track_error, CVSEQ, true)){
    for(int i=0; i<n_bundles; i++)
      multi_marker_bundles[i]->Update(marker_detector.markers, calibratedCam, bundlePoses[i]);
    
    if(marker_detector.DetectAdditional(image, calibratedCam, false) > 0){
      for(int i=0; i<n_bundles; i++){
  if ((multi_marker_bundles[i]->SetTrackMarkers(marker_detector, calibratedCam, bundlePoses[i], image) > 0))
    multi_marker_bundles[i]->Update(marker_detector.markers, calibratedCam, bundlePoses[i]);
      }
    }
  }
}

// Updates the bundlePoses of the multi_marker_bundles by detecting markers and
// using all markers in a bundle to infer the master tag's position
void GetMultiMarkerPoses(IplImage *image, ARCloud &cloud) {
  //Detect and track the markers
  if (kinect_marker_detector.Detect(image, kinectCam, true, false, max_new_marker_error,
           max_track_error, CVSEQ, true)) 
    {
      //printf("\n--------------------------\n\n");
      for (size_t i=0; i<kinect_marker_detector.markers->size(); i++)
      {
      vector<cv::Point, Eigen::aligned_allocator<cv::Point> > pixels;
      Marker *m = &((*kinect_marker_detector.markers)[i]);
      int id = m->GetId();
      
            
      //Get the 3D inner corner points - more stable than outer corners that can "fall off" object
      int resol = m->GetRes();
      int ori = m->ros_orientation;
        
      PointDouble pt1, pt2, pt3, pt4;
      pt4 = m->ros_marker_points_img[0];
      pt3 = m->ros_marker_points_img[resol-1];
      pt1 = m->ros_marker_points_img[(resol*resol)-resol];
      pt2 = m->ros_marker_points_img[(resol*resol)-1];
      
      m->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
      m->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
      m->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
      m->ros_corners_3D[3] = cloud(pt4.x, pt4.y);
      
      if(ori >= 0 && ori < 4){
        if(ori != 0){
          std::rotate(m->ros_corners_3D.begin(), m->ros_corners_3D.begin() + ori, m->ros_corners_3D.end());
        }
      }
      else
        ROS_ERROR("FindMarkerBundles: Bad Orientation: %i for ID: %i", ori, id);

      //Get the 3D marker points
      BOOST_FOREACH (const PointDouble& p, m->ros_marker_points_img)
        pixels.push_back(cv::Point(p.x, p.y));    
      ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

      //Use the kinect data to find a plane and pose for the marker
      int ret = PlaneFitPoseImprovement(i, m->ros_corners_3D, selected_points, cloud, m->pose);
      m->valid = (ret >= 0);
    } 

    //For each master tag, infer the 3D position of its corners from other visible tags
    //Then, do a plane fit to those new corners     
    ARCloud inferred_corners;
    for(int i=0; i<n_bundles; i++){
      if(bundles_seen[i] > 0){
        for (size_t j=0; j<marker_detector.markers->size(); j++){
            Marker *m = &((*marker_detector.markers)[j]);                     
            if(m->GetId() == master_id[i])
                bundlePoses[i] = m->pose;
        } 

        Pose ret_pose;
        if(med_filt_size > 0){
            med_filts[i]->addPose(bundlePoses[i]);
            med_filts[i]->getMedian(ret_pose);
            kinectBundlePoses[i] = ret_pose;
        }   
      }   
    }
  }
}

void makeMarkerMsgs(int type, int id, Pose &p, sensor_msgs::ImageConstPtr image_msg, tf::StampedTransform &CamToOutput, visualization_msgs::Marker *rvizMarker, ar_track_alvar::AlvarMarker *ar_pose_marker){
  double px,py,pz,qx,qy,qz,qw;
  
  px = p.translation[0]/100.0;
  py = p.translation[1]/100.0;
  pz = p.translation[2]/100.0;
  qx = p.quaternion[1];
  qy = p.quaternion[2];
  qz = p.quaternion[3];
  qw = p.quaternion[0];

  //Get the marker pose in the camera frame
  tf::Quaternion rotation (qx,qy,qz,qw);
  tf::Vector3 origin (px,py,pz);
  tf::Transform t (rotation, origin);  //transform from cam to marker

  tf::Vector3 markerOrigin (0, 0, 0);
  tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
  tf::Transform markerPose = t * m;

  //Publish the cam to marker transform for main marker in each bundle
  if(type==MAIN_MARKER){
    std::string markerFrame = "ar_marker_";
    std::stringstream out;
    out << id;
    std::string id_string = out.str();
    markerFrame += id_string;
    tf::StampedTransform camToMarker (t, ros::Time::now(), image_msg->header.frame_id, markerFrame.c_str());
    tf_broadcaster->sendTransform(camToMarker);
  }

  //Create the rviz visualization message
  tf::poseTFToMsg (markerPose, rvizMarker->pose);
  rvizMarker->header.frame_id = image_msg->header.frame_id;
  rvizMarker->header.stamp = image_msg->header.stamp;
  rvizMarker->id = id;

  rvizMarker->scale.x = 1.0 * marker_size/100.0;
  rvizMarker->scale.y = 1.0 * marker_size/100.0;
  rvizMarker->scale.z = 0.2 * marker_size/100.0;

  if(type==MAIN_MARKER)
    rvizMarker->ns = "main_shapes";
  else
    rvizMarker->ns = "basic_shapes";


  rvizMarker->type = visualization_msgs::Marker::CUBE;
  rvizMarker->action = visualization_msgs::Marker::ADD;

  //Determine a color and opacity, based on marker type
  if(type==MAIN_MARKER){
    rvizMarker->color.r = 1.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 1.0;
  }
  else if(type==VISIBLE_MARKER){
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 1.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 0.7;
  }
  else if(type==GHOST_MARKER){
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 1.0f;
    rvizMarker->color.a = 0.5;
  }

  rvizMarker->lifetime = ros::Duration (1.0);

  // Only publish the pose of the master tag in each bundle, since that's all we really care about aside from visualization 
  if(type==MAIN_MARKER){
    //Take the pose of the tag in the camera frame and convert to the output frame (usually torso_lift_link for the PR2)
    tf::Transform tagPoseOutput = CamToOutput * markerPose;

    //Create the pose marker message
    tf::poseTFToMsg (tagPoseOutput, ar_pose_marker->pose.pose);
    ar_pose_marker->header.frame_id = output_frame;
    ar_pose_marker->header.stamp = image_msg->header.stamp;
    ar_pose_marker->id = id;
  }
  else
    ar_pose_marker = NULL;
}

void addTransformToAverage(tf::Transform incoming, std::deque<tf::Transform> &transforms, tf::Transform &transform)
{
    transforms.push_back(incoming); 
    int n = transforms.size();
    if (n <=1)
    {
         transform = tf::Transform(incoming);
         return;
    }
    tf::Quaternion quat = transform.getRotation();
    tf::Vector3 vec = transform.getOrigin();
    
    if (n > 100)
    {
        double t = 1.0 / (n - 1);
        tf::Transform outgoing = transforms.front();
        transforms.pop_front();

        tf::Quaternion oQuat = outgoing.getRotation();
        tf::Vector3 oVec = outgoing.getOrigin();
           
        
        vec = (1+t) * vec - t * oVec;
        n--;
    }
    double t = 1.0 / (n-1);
    
    
    tf::Quaternion iQuat = incoming.getRotation();
    tf::Vector3 iVec = incoming.getOrigin();

    transform.setOrigin((1 - t) * vec + t * iVec);
    transform.setRotation(quat.slerp(iQuat, t));
}


//Callback to handle getting video frames and processing them
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
  //If we've already gotten the cam info, then go ahead
  if(calibratedCam->getCamInfo_){
    try{
      //Get the transformation from the Camera to the output frame for this image capture
      tf::StampedTransform CamToOutput;
      try{
        ros::Time now = ros::Time::now();
  tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, now, ros::Duration(1.0));
  tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, now, CamToOutput);
      }
      catch (tf::TransformException ex){
  ROS_ERROR("%s",ex.what());
      }

      visualization_msgs::Marker rvizMarker;
      ar_track_alvar::AlvarMarker ar_pose_marker;
      
      //Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      //Get the estimated pose of the main markers by using all the markers in each bundle

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      IplImage ipl_image = cv_ptr_->image;
      GetMultiMarkerPoses(&ipl_image);
    
      //Draw the observed markers that are visible and note which bundles have at least 1 marker seen
      for(int i=0; i<n_bundles; i++)
      {
  bundles_seen[i]++;
      }
      for (size_t i=0; i<marker_detector.markers->size(); i++)
  {
    int id = (*(marker_detector.markers))[i].GetId();

    // Draw if id is valid
    if(id >= 0){

      //Mark the bundle that marker belongs to as "seen"
      for(int j=0; j<n_bundles; j++){
        for(int k=0; k<bundle_indices[j].size(); k++){
    if(bundle_indices[j][k] == id){
      bundles_seen[j]++;
      break;
    }
        }
      }

      // Don't draw if it is a master tag...we do this later, a bit differently
      bool should_draw = true;
      for(int j=0; j<n_bundles; j++){
        if(id == master_id[j]) should_draw = false;
      }
      if(should_draw){
        Pose p = (*(marker_detector.markers))[i].pose;
        makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
        rvizMarkerPub_.publish (rvizMarker);
      }
    }
  }
      
      //Draw the main markers, whether they are visible or not -- but only if at least 1 marker from their bundle is currently seen
      for(int i=0; i<n_bundles; i++)
  {
    if(bundles_seen[i] > 0){
      makeMarkerMsgs(MAIN_MARKER, master_id[i], bundlePoses[i], image_msg, CamToOutput, &rvizMarker, &ar_pose_marker);
      rvizMarkerPub_.publish (rvizMarker);
      
            //Get the pose relative to the camera
            int id = i;
            Pose p = bundlePoses[i];

            double px = p.translation[0]/100.0;
            double py = p.translation[1]/100.0;
            double pz = p.translation[2]/100.0;
            double qx = p.quaternion[1];
            double qy = p.quaternion[2];
            double qz = p.quaternion[3];
            double qw = p.quaternion[0];

            tf::Quaternion rotation (qx,qy,qz,qw);
            tf::Vector3 origin (px,py,pz);
            tf::Transform t (rotation, origin);
            tf::Vector3 markerOrigin (0, 0, 0);
            tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
            tf::Transform markerPose = t * m; // marker pose in the camera frame

            if (camTransforms.size() <= i)
            {
                std::deque<tf::Transform> transformDeque;
                camTransforms.push_back(transformDeque);
                tf::Quaternion p (0,0,0,1);
                tf::Vector3 o (0,0,0);
                tf::Transform t (p, o);
                camTransform.push_back(t);
            }
            addTransformToAverage(markerPose, camTransforms[i], camTransform[i]);

            std::stringstream out;
            out << "ar_marker_" << id;
            std::string markerFrame = out.str();
            tf::StampedTransform camToMarker (camTransform[i], ros::Time::now(), calibratedFrameID.c_str(), markerFrame);
            tf_broadcaster->sendTransform(camToMarker);
            
          }
        }
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    }
  }
}

//Callback to handle getting kinect point clouds and processing them
void getPointCloudCallback (const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

  //If we've already gotten the cam info, then go ahead
  if(kinectCam->getCamInfo_){
    try{
      //Convert cloud to PCL 
      ARCloud cloud;
      pcl::fromROSMsg(*msg, cloud);

      //Get an OpenCV image from the cloud
      pcl::toROSMsg (*msg, *image_msg);
      image_msg->header.stamp = msg->header.stamp;
      image_msg->header.frame_id = msg->header.frame_id;
            
      //Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      //Get the estimated pose of the main markers by using all the markers in each bundle

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      IplImage ipl_image = cv_ptr_->image;
      GetMultiMarkerPoses(&ipl_image, cloud);
      ros::Time now = ros::Time::now();
      for (size_t i=0; i<kinect_marker_detector.markers->size(); i++)
      {
          int id = (*(kinect_marker_detector.markers))[i].GetId();
                Pose p = (*(kinect_marker_detector.markers))[i].pose;
          // Draw if id is valid
          double px = p.translation[0]/100.0;
          double py = p.translation[1]/100.0;
          double pz = p.translation[2]/100.0;
          double qx = p.quaternion[1];
          double qy = p.quaternion[2];
          double qz = p.quaternion[3];
          double qw = p.quaternion[0];
 
          tf::Quaternion rotation (qx, qy, qz, qw);
          tf::Vector3 origin (px,py,pz);
          tf::Transform t (rotation, origin);
          tf::Vector3 markerOrigin (0, 0, 0);
          tf::Quaternion quat;
          quat.setRPY(0,0,-3.14159/2.0);
          tf::Transform m (quat, markerOrigin);
          tf::Transform markerPose = t * m; // marker pose in the camera frame

          tf::StampedTransform camBaseToCamera;
          try{
            
            tf_listener->waitForTransform(kinectBaseLinkFrameID, image_msg->header.frame_id, now, ros::Duration(1.0));
            tf_listener->lookupTransform(kinectBaseLinkFrameID, image_msg->header.frame_id, now, camBaseToCamera);
          }
          catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
          }
         
          std::stringstream out;
          out << "ar_marker_" << id;
          std::string markerFrame = out.str();
          tf::Transform linkToMarker = camBaseToCamera * markerPose;
          addTransformToAverage(linkToMarker.inverse(), kinectTransforms, kinectTransform);
          tf::StampedTransform markerToCamera (kinectTransform, now, markerFrame, kinectBaseLinkFrameID);
          tf_broadcaster->sendTransform(markerToCamera);
  }
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR ("ar_track_alvar: Image error: %s", image_msg->encoding.c_str ());
    }
  } 
  else
  {
     ROS_WARN_STREAM("Can't get kinect info on topic " << kinectInfoTopic);
  }
}


//Create a ROS frame out of the known corners of a tag in the weird marker coord frame used by Alvar markers (x right y forward z up)
//p0-->p1 should point in Alvar's pos X direction
//p1-->p2 should point in Alvar's pos Y direction
int makeMasterTransform (const CvPoint3D64f& p0, const CvPoint3D64f& p1,
                         const CvPoint3D64f& p2, const CvPoint3D64f& p3,
                         tf::Transform &retT)
  {
    const tf::Vector3 q0(p0.x, p0.y, p0.z);
    const tf::Vector3 q1(p1.x, p1.y, p1.z);
    const tf::Vector3 q2(p2.x, p2.y, p2.z);
    const tf::Vector3 q3(p3.x, p3.y, p3.z);
  
    // (inverse) matrix with the given properties
    const tf::Vector3 v = (q1-q0).normalized();
    const tf::Vector3 w = (q2-q1).normalized();
    const tf::Vector3 n = v.cross(w);
    tf::Matrix3x3 m(v[0], v[1], v[2], w[0], w[1], w[2], n[0], n[1], n[2]);
    m = m.inverse();
    
    //Translate to quaternion
    if(m.determinant() <= 0)
        return -1;
  
    //Use Eigen for this part instead, because the ROS version of bullet appears to have a bug
    Eigen::Matrix3f eig_m;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            eig_m(i,j) = m[i][j];
        }
    }
    Eigen::Quaternion<float> eig_quat(eig_m);
    
    // Translate back to bullet
    tfScalar ex = eig_quat.x();
    tfScalar ey = eig_quat.y();
    tfScalar ez = eig_quat.z();
    tfScalar ew = eig_quat.w();
    tf::Quaternion quat(ex,ey,ez,ew);
    quat = quat.normalized();
    
    double qx = (q0.x() + q1.x() + q2.x() + q3.x()) / 4.0;
    double qy = (q0.y() + q1.y() + q2.y() + q3.y()) / 4.0;
    double qz = (q0.z() + q1.z() + q2.z() + q3.z()) / 4.0;
    tf::Vector3 origin (qx,qy,qz);
    
    tf::Transform tform (quat, origin);  //transform from master to marker
    retT = tform;
    
    return 0;
  }

//Find the coordinates of the Master marker with respect to the coord frame of each of it's child markers
//This data is used for later estimation of the Master marker pose from the child poses
int calcAndSaveMasterCoords(MultiMarkerBundle &master)
{
    int mast_id = master.master_id;
    std::vector<tf::Vector3> rel_corner_coords;
    
    //Go through all the markers associated with this bundle
    for (size_t i=0; i<master.marker_indices.size(); i++){
        int mark_id = master.marker_indices[i];
        rel_corner_coords.clear();
        
        //Get the coords of the corners of the child marker in the master frame
        CvPoint3D64f mark_corners[4];
        for(int j=0; j<4; j++){
            mark_corners[j] = master.pointcloud[master.pointcloud_index(mark_id, j)];
        }
        
        //Use them to find a transform from the master frame to the child frame
        tf::Transform tform;
        makeMasterTransform(mark_corners[0], mark_corners[1], mark_corners[2], mark_corners[3], tform);
    
        //Finally, find the coords of the corners of the master in the child frame
        for(int j=0; j<4; j++){
            
            CvPoint3D64f corner_coord = master.pointcloud[master.pointcloud_index(mast_id, j)];
            double px = corner_coord.x;
            double py = corner_coord.y;
            double pz = corner_coord.z;
        
            tf::Vector3 corner_vec (px, py, pz);
            tf::Vector3 ans = (tform.inverse()) * corner_vec;
            rel_corner_coords.push_back(ans);
        }
        
        master.rel_corners.push_back(rel_corner_coords);
    }
    
    return 0;
}


int main(int argc, char *argv[])
{
  ros::init (argc, argv, "marker_detect");
  ros::NodeHandle n;

  if(argc < 9){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./bundle_calibrate <output file> <marker size in cm> <max new marker error> <max track error> <kinect image topic> <kinect info topic> <kinect base link frame ID> <calibrated image topic> <calibrated info topic> <calibrated frame ID> <output frame> <median filt size> <list of bundle XML files...>" << endl;
    std::cout << std::endl;
    return 0;
  }

  // Get params from command line
  
  launchFileName = argv[1];
  marker_size = atof(argv[2]);
  max_new_marker_error = atof(argv[3]);
  max_track_error = atof(argv[4]);
  kinectImageTopic = argv[5]; 
  kinectInfoTopic = argv[6];
  kinectBaseLinkFrameID = argv[7];
  calibratedImageTopic = argv[8];
  calibratedInfoTopic = argv[9];
  calibratedFrameID = argv[10];
  output_frame = argv[11];
  med_filt_size = atoi(argv[12]);
  int n_args_before_list = 13;
  n_bundles = argc - n_args_before_list;

  marker_detector.SetMarkerSize(marker_size);
  kinect_marker_detector.SetMarkerSize(marker_size);
  multi_marker_bundles = new MultiMarkerBundle*[n_bundles]; 
  bundlePoses = new Pose[n_bundles];
  kinectBundlePoses = new Pose[n_bundles];
  master_id = new int[n_bundles]; 
  bundle_indices = new std::vector<int>[n_bundles]; 
  bundles_seen = new int[n_bundles]; 
  master_visible = new bool[n_bundles];

  //Create median filters
  med_filts = new ata::MedianFilter*[n_bundles];
  for(int i=0; i<n_bundles; i++)
    med_filts[i] = new ata::MedianFilter(med_filt_size);

  // Load the marker bundle XML files
  for(int i=0; i<n_bundles; i++){
    kinectBundlePoses[i].Reset(); 
    bundlePoses[i].Reset();   
    MultiMarker loadHelper;
    if(loadHelper.Load(argv[i + n_args_before_list], FILE_FORMAT_XML)){
      vector<int> id_vector = loadHelper.getIndices();
      multi_marker_bundles[i] = new MultiMarkerBundle(id_vector); 
      multi_marker_bundles[i]->Load(argv[i + n_args_before_list], FILE_FORMAT_XML);
      master_id[i] = multi_marker_bundles[i]->getMasterId();
      bundle_indices[i] = multi_marker_bundles[i]->getIndices();
      calcAndSaveMasterCoords(*(multi_marker_bundles[i]));
    }
    else{
      cout<<"Cannot load file "<< argv[i + n_args_before_list] << endl; 
      return 0;
    }   
  }  

  // Set up camera, listeners, and broadcasters
  kinectCam = new Camera(n, kinectInfoTopic);
  calibratedCam = new Camera(n, calibratedInfoTopic);
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();
  rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
  rvizMarkerPub2_ = n.advertise < visualization_msgs::Marker > ("visualization_marker_calibrated", 0);
  
  //Give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();      
   
  //Subscribe to topics and set up callbacks
  ROS_INFO_STREAM ("Subscribing to kinect image topic '" << kinectImageTopic << "'");
  cloud_sub_ = n.subscribe(kinectImageTopic, 1, &getPointCloudCallback);

  image_transport::ImageTransport it_(n);
//Subscribe to topics and set up callbacks
  ROS_INFO_STREAM ("Subscribing to calibrated image topic '" << calibratedImageTopic << "'");
  calibratedSub_ = it_.subscribe(calibratedImageTopic, 1, &getCapCallback);

  ros::spin();

  return 0;
}