#ifndef __OBJECT_SCENE_INTERFACE__
#define __OBJECT_SCENE_INTERFACE__

// ROS
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Baxter Utilities
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "baxter_grasps_server/GraspService.h"
#include "ros/time.h"
 
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/DigitalIOState.h"

#include "ros/console.h"

#include <iostream>

#include <map>
#include <vector>
#include <algorithm>
#include <iterator>
#include <string>
#include <utility> // header for pair
#include <math.h>

static ros::Duration OBJECT_TIMEOUT(30.0);

class ObjectSceneInterface
{
  
private:
	std::vector<std::string> frozenObjects;
        moveit::planning_interface::PlanningSceneInterface scene;
	std::map<std::string, geometry_msgs::PoseStamped> objectPoses;
	std::map<std::string, ros::Time> lastTimeSeen;
	tf::TransformListener transformer;
	
	geometry_msgs::PoseStamped getPoseStampedFromPoseWithCovariance(geometry_msgs::PoseWithCovarianceStamped pose)
	{
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header = pose.header;
		poseStamped.pose.position = pose.pose.pose.position;
		poseStamped.pose.orientation = pose.pose.pose.orientation;
		return poseStamped;
	}

	void addStaticObjects(std::vector<moveit_msgs::CollisionObject> &collisionObjects)
	{
		//ROS_INFO("Kinect");
		geometry_msgs::PoseStamped kinectPose;
		kinectPose.header.frame_id = "/camera_link";
		kinectPose.pose.orientation.w = 1.0;
		//ROS_INFO("Kinect");
		moveit_msgs::CollisionObject kinect = this->createCollisionObject( kinectPose.pose, "kinect", kinectPose.header, 0.1, 0.3, 0.05);
		collisionObjects.push_back(kinect);


		//ROS_INFO("tripod");
		
		geometry_msgs::PoseStamped tripodPose;
		tripodPose.header.frame_id = "/world";
		tripodPose.pose.position.x = 0.95;
		tripodPose.pose.position.y = 0.0;
		tripodPose.pose.position.z = 0.0;
		tripodPose.pose.orientation.w = 1.0;
		moveit_msgs::CollisionObject tripod = this->createCollisionObject(tripodPose.pose, "tripod", tripodPose.header, 0.15, 2.0, 1.0);
		collisionObjects.push_back(tripod);

		//ROS_INFO("sink");
		/*
		geometry_msgs::PoseStamped sinkPose;
		sinkPose.header.frame_id = "/world";
		sinkPose.pose.position.x = 0.5;
		sinkPose.pose.position.y = 0.7;
		sinkPose.pose.position.z = -0.05;
		sinkPose.pose.orientation.w = 1.0;
		moveit_msgs::CollisionObject sink = this->createCollisionObject(sinkPose.pose, "sink", sinkPose.header, 0.39, 0.29, 0.22);
		collisionObjects.push_back(sink);*/

		
				
		geometry_msgs::PoseStamped tablePose;
		tablePose.header.frame_id = "/world";
                // side table
		tablePose.pose.position.x = -0.37;
		tablePose.pose.position.y = 0.55;
		tablePose.pose.position.z = -0.22;  // conservative height

                // front table
		//tablePose.pose.position.x = 0.7;
		//tablePose.pose.position.y = 0.0;
		//tablePose.pose.position.z = -0.22;  // conservative height
                
                //tablePose.pose.position.z = -0.3;  // lower height
		tablePose.pose.orientation.w = 1.0;
		moveit_msgs::CollisionObject table = this->createCollisionObject(tablePose.pose, "table", tablePose.header, 1.0, 2.0, 0.2);
		collisionObjects.push_back(table);
	}

	

	moveit_msgs::CollisionObject createCollisionObject(geometry_msgs::Pose pose, std::string objectName, std_msgs::Header header, double width = 0.03, double length = 0.03, double height = 0.2)
	{
		//ROS_INFO("Create collision object");
		moveit_msgs::CollisionObject collisionObject;
		collisionObject.header = header;
		collisionObject.id = objectName;
		
		collisionObject.primitives.resize(1);
		collisionObject.primitive_poses.resize(1);

		collisionObject.primitive_poses[0] = pose;
		collisionObject.operation = moveit_msgs::CollisionObject::ADD;
		
		std::string *error;
		ros::Time when;
		std::string objectFrame = "/world";
		//ROS_INFO("object frame");
		if (objectName.compare("flour_bowl") == 0)
		{
			objectFrame = "ar_marker_1";
		}
		if (objectName.compare("cocoa_bowl") == 0)
		{
			objectFrame = "ar_marker_0";
		}
		if (objectName.compare("butter_bowl") == 0)
		{
			objectFrame = "ar_marker_7";
		}
		if (objectName.compare("white_sugar_bowl") == 0)
		{
			objectFrame = "ar_marker_5";
		}
		if (objectName.compare("whisk") == 0)
		{
			objectFrame = "ar_marker_9";
		}
		if (objectName.compare("spoon") == 0)
		{
			objectFrame = "ar_marker_10";
		}
		if (objectName.compare("mixing_bowl_1") == 0)
		{
			objectFrame = "ar_marker_8";
		}
		if (objectName.compare("mixing_bowl_2") == 0)
		{
			objectFrame = "ar_marker_11";
		}

		geometry_msgs::PoseStamped poseStamped, poseOut;
		

		//ROS_INFO_STREAM("Object frame " << objectFrame);

		if (objectFrame.compare(header.frame_id) != 0)
		{
			//ROS_INFO_STREAM("Transform frame " << objectFrame << ", " << header.frame_id);
		
			try
			{
				this->transformer.getLatestCommonTime(objectFrame, header.frame_id, when, error);
				poseStamped.header = header;
				poseStamped.header.stamp = when;
				poseStamped.pose = pose;
				
				this->transformer.transformPose(objectFrame, poseStamped, poseOut);
				collisionObject.header = poseOut.header;

				collisionObject.primitive_poses[0] = poseOut.pose;	
			}
			catch (...)
			{
				collisionObject.header = header;
				collisionObject.primitive_poses[0] = pose;
				
				collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
				collisionObject.primitives[0].dimensions.resize(3);
				collisionObject.primitives[0].dimensions[0] = width;
				collisionObject.primitives[0].dimensions[1] = length;
				collisionObject.primitives[0].dimensions[2] = height;
				return collisionObject;
			}
		}
		else
		{
			poseOut.header = header;
			poseOut.header.stamp = ros::Time::now();
			poseOut.pose = pose;
		}


		if (objectName.compare("flour_bowl") == 0 || objectName.compare("cocoa_bowl") == 0 )
		{
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
			collisionObject.primitives[0].dimensions.resize(2);
			collisionObject.primitives[0].dimensions[0] = 0.07;
			collisionObject.primitives[0].dimensions[1] = 0.07;

			
			collisionObject.primitive_poses[0].position.x += 0.08;
			collisionObject.primitive_poses[0].position.y += 0.045;
			collisionObject.primitive_poses[0].position.z += 0.07/2.0;
		
		}
		else if (objectName.compare("butter_bowl") == 0)
		{

			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
			collisionObject.primitives[0].dimensions.resize(2);
			collisionObject.primitives[0].dimensions[0] = 0.06;
			collisionObject.primitives[0].dimensions[1] = 0.13/2.0;

			collisionObject.primitive_poses[0].position.x += 0.09;
			collisionObject.primitive_poses[0].position.y += 0.03;
			collisionObject.primitive_poses[0].position.z += 0.07/2.0;
		}
		else if (objectName.compare("white_sugar_bowl") == 0)
		{
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
			collisionObject.primitives[0].dimensions.resize(2);
			collisionObject.primitives[0].dimensions[0] = 0.06;
			collisionObject.primitives[0].dimensions[1] = 0.13/2.0;

			collisionObject.primitive_poses[0].position.x += 0.075;
			collisionObject.primitive_poses[0].position.y += 0.04;
			collisionObject.primitive_poses[0].position.z += 0.07/2.0;
		}

		else if (objectName.compare("whisk") == 0)
		{
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
			collisionObject.primitives[0].dimensions.resize(3);
			collisionObject.primitives[0].dimensions[0] = 0.27;
			collisionObject.primitives[0].dimensions[1] = 0.13;
			collisionObject.primitives[0].dimensions[2] = 0.06;

			collisionObject.primitive_poses[0].position.x += 0.16;
			collisionObject.primitive_poses[0].position.y += 0.04;
			collisionObject.primitive_poses[0].position.z += 0.06/2.0;
		}
		else if (objectName.compare("spoon") == 0)
		{
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
			collisionObject.primitives[0].dimensions.resize(3);
			collisionObject.primitives[0].dimensions[0] = 0.01;
			collisionObject.primitives[0].dimensions[1] = 0.01;
			collisionObject.primitives[0].dimensions[2] = 0.015;

			collisionObject.primitive_poses[0].position.x += 0.04;
			collisionObject.primitive_poses[0].position.y += 0.1;
			collisionObject.primitive_poses[0].position.z += 0.015/2.0;
		} else 
		{
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
			collisionObject.primitives[0].dimensions.resize(3);
			collisionObject.primitives[0].dimensions[0] = width;
			collisionObject.primitives[0].dimensions[1] = length;
			collisionObject.primitives[0].dimensions[2] = height;
		}


		//ROS_INFO_STREAM("pose frame " << poseOut.header.frame_id);
		if (poseOut.header.frame_id.compare("world") != 0 || poseOut.header.frame_id.compare("/world") != 0)
		{
			//ROS_INFO_STREAM("Transform frame " << poseOut.header.frame_id << " " << when.toSec() );
			try
			{

				this->transformer.getLatestCommonTime("world", poseOut.header.frame_id, when, error);
				
				
				geometry_msgs::PoseStamped poseFinal;
				//ROS_INFO("Here");
			poseStamped.header = poseOut.header;
				//ROS_INFO("Here");
			poseStamped.header.stamp = when;
				//ROS_INFO("Here");
			poseStamped.pose = collisionObject.primitive_poses[0];

				//ROS_INFO("Transforming pose");
			
				//ROS_INFO("Here");
			this->transformer.transformPose("world", poseStamped, poseFinal);
				//ROS_INFO("Here");
			collisionObject.header = poseFinal.header;

				//ROS_INFO("Here");
			collisionObject.primitive_poses[0] = poseFinal.pose;		
			}
			catch (...)
			{
				//ROS_INFO("Here");
			collisionObject.header = header;
				//ROS_INFO("Here");
			collisionObject.primitive_poses[0] = pose;
				
				//ROS_INFO("Here");
			collisionObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
				//ROS_INFO("Here");
			collisionObject.primitives[0].dimensions.resize(3);
				//ROS_INFO("Here");
			collisionObject.primitives[0].dimensions[0] = width;
				//ROS_INFO("Here");
			collisionObject.primitives[0].dimensions[1] = length;
				//ROS_INFO("Here");
			collisionObject.primitives[0].dimensions[2] = height;
			}
		}

		//ROS_INFO("Returning");
		return collisionObject;
	}

	void removeUnobservedObjects(std::vector<std::string> observedObjects)
	{
		observedObjects.push_back("kinect");
		observedObjects.push_back("tripod");
		observedObjects.push_back("table");
		observedObjects.push_back("sink");
		std::vector<std::string> unobservedObjects, toRemove;
		std::vector<std::string> planningSceneObjects = this->scene.getKnownObjectNames();

		std::sort(observedObjects.begin(), observedObjects.end());
		std::sort(planningSceneObjects.begin(), planningSceneObjects.end());

		std::set_difference(planningSceneObjects.begin(), planningSceneObjects.end(),
							observedObjects.begin(), observedObjects.end(),
							std::back_inserter(unobservedObjects));


		for (std::vector<std::string>::iterator beginIt = unobservedObjects.begin(),
			 endIt = unobservedObjects.end();
			 beginIt != endIt; beginIt++ )
		{
			bool shouldRemove = false;
			ros::Duration timeSinceSeen(0.0);
			std::map<std::string, ros::Time>::iterator findIt = this->lastTimeSeen.find(*beginIt);
			if (findIt == this->lastTimeSeen.end())
			{
				shouldRemove = true;
				
			}
			else
			{
				ros::Duration timeSinceSeen = ros::Time::now() - findIt->second;
				shouldRemove |= (timeSinceSeen.toSec() > OBJECT_TIMEOUT.toSec());
				
			}

			if (shouldRemove)
			{
				//ROS_INFO_STREAM("Object " << *beginIt << " has not been seen in awhile. Removing object from collision scene");
				toRemove.push_back(*beginIt);
			}

		}

		this->scene.removeCollisionObjects(toRemove);
	}

public:
	void addObjects()
	{
          std::vector<moveit_msgs::CollisionObject> collisionObjects;
          this->addStaticObjects(collisionObjects);
          this->scene.addCollisionObjects(collisionObjects);
	}

	void addObjects(std::vector<object_recognition_msgs::RecognizedObject> objects)
	{
          std::vector<moveit_msgs::CollisionObject> collisionObjects;

          this->addStaticObjects(collisionObjects);
		/* for (int i = 0; i < objects.size(); i++) */
		/* { */
		/* 	object_recognition_msgs::RecognizedObject object = objects[i]; */
			
		/* 	if (std::find(frozenObjects.begin(), frozenObjects.end(), object.type.key) == frozenObjects.end()) */
		/* 	{ */
		/* 		if (object.type.key.compare("mixing_bowl_1") != 0 && object.type.key.compare("mixing_bowl_2") != 0 ) */
		/* 		{ */
		/* 			moveit_msgs::CollisionObject collisionObject = this->createCollisionObject( object.pose.pose.pose, object.type.key, object.pose.header); */
		/* 			collisionObjects.push_back(collisionObject); */
		/* 		} */
		/* 	} */
		/* } */


		//ROS_INFO("Adding objcts to scene");
		this->scene.addCollisionObjects(collisionObjects);
	}

	void freezeObject(std::string object)
	{
		this->frozenObjects.clear();
		this->frozenObjects.push_back(object);
	}

	void freezeObjects(std::vector<std::string> objects) 
	{
		this->frozenObjects.clear();
		this->frozenObjects.insert(this->frozenObjects.begin(), objects.begin(), objects.end());
	};

	void freezeAllObjects()
	{
		this->frozenObjects.clear();
		std::map<std::string, ros::Time>::iterator it, endIt;
		for (it = this->lastTimeSeen.begin(), endIt = this->lastTimeSeen.end(); it != endIt; it++)
		{
			this->frozenObjects.push_back(it->first);
		}
	}


	void clearFrozenObjects() {this->frozenObjects.clear();}


	void updateObjects(const object_recognition_msgs::RecognizedObjectArray::ConstPtr &msg)
	{
		this->objectPoses.clear();
		std::vector<std::string> observedObjects;
		for (int i = 0; i < msg->objects.size(); i++)
		{
			std::string objectName = msg->objects[i].type.key;
			this->lastTimeSeen[objectName] = ros::Time::now();
			geometry_msgs::PoseStamped pose = this->getPoseStampedFromPoseWithCovariance(msg->objects[i].pose);
			this->objectPoses[objectName] = pose;
			observedObjects.push_back(objectName);
		}	

		this->addObjects(msg->objects);
		this->removeUnobservedObjects(observedObjects);
	}

        void removeAllObjects() 
        {
          std::vector<std::string> objects = scene.getKnownObjectNames(); 
          scene.removeCollisionObjects(objects);
        }
        void setStaticObjects()
        {
          removeAllObjects();
          std::vector<moveit_msgs::CollisionObject> collisionObjects;
          addStaticObjects(collisionObjects);
          scene.addCollisionObjects(collisionObjects);
        }

};

#endif 
