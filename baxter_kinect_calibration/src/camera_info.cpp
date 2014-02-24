#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include <camera_info_manager/camera_info_manager.h>

camera_info_manager::CameraInfoManager *cameraInfoManager;
ros::Subscriber cameraInfoSubscriber;
ros::Publisher cameraInfoPublisher;
void cameraInfoCallback(const sensor_msgs::CameraInfoPtr &msg)
{
  sensor_msgs::CameraInfoPtr
    ci(new sensor_msgs::CameraInfo(cameraInfoManager->getCameraInfo()));	
  if (true)
  {
  cameraInfoPublisher.publish(ci);
  }
  else
  {
  cameraInfoPublisher.publish(msg);
  }  
}

int main(int argc, char ** argv)
{
  if (argc <= 2)
  {
      std::cout << "Usage: camera_info_publisher <new camera namespace> <old camera info topic> <yaml calibration file>" << std::endl;
      return 0;
  }
  ros::init(argc, argv, "baxter_camera_info");
  ros::NodeHandle nh(argv[1]);
  if (argc <= 3)
  {
     cameraInfoManager = new camera_info_manager::CameraInfoManager(nh, argv[1]);
  }
  else
  {
     cameraInfoManager = new camera_info_manager::CameraInfoManager(nh, argv[1], argv[3]);
  }
  cameraInfoSubscriber = nh.subscribe(argv[2], 1000, cameraInfoCallback);
  cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
  ros::spin();
}
