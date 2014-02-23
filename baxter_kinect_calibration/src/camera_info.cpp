#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include <camera_info_manager/camera_info_manager.h>

camera_info_manager::CameraInfoManager *camera_info_manager;
ros::Subscriber cameraInfoSubscriber;
void cameraInfoCallback(const sensor_msgs::CameraInfoPtr &msg)
{
  sensor_msgs::CameraInfoPtr
    ci(new sensor_msgs::CameraInfo(camera_info_manager->getCameraInfo());
}
void PublishCameraInfo(
  ros::Publisher camera_info_publisher)
{
  sensor_msgs::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;

  camera_info_publisher.publish(camera_info_msg);
}

int main(int argc, char ** argv)
{
  if (argc <= 2)
  {
      std::cout << "Usage: camera_info_publisher <yaml calibration file> <old camera info topic>" << std::endl;
      return 0;
  }
  ros::init(argv, argc, "baxter_camera_info");
  ros::NodeHandle nh;
  camera_info_manager = new CameraInfoManager(nh, "baxter_left_camera", argv[1]);
  cameraInfoSubscriber = nh.subscribe(argv[2], 1000, cameraInfoCallback);
  nh
}
