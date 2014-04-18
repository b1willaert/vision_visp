/****************************************************************************
 *
 * $Id: file.h 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Client node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file client.cpp
  \brief Client node calling a quick compute service, a compute service and 2 publishing to world_effector_topic and camera_object_topic.
*/

#include "client.h"
#include <geometry_msgs/Transform.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include "visp_hand2eye_calibration/TwoTransformArrays.h"

#include <visp_bridge/3dpose.h>
#include "names.h"

#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>



namespace visp_hand2eye_calibration
{
Client::Client()
{
	desired_endeffector_poses_
      = n_.advertise<visp_hand2eye_calibration::TransformArray> (visp_hand2eye_calibration::desired_robot_poses_topic, 1000);

  reset_service_
      = n_.serviceClient<visp_hand2eye_calibration::reset> (visp_hand2eye_calibration::reset_service);
  compute_transform_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> (
                                                                                            visp_hand2eye_calibration::compute_effector_camera_quick_service);
}

void Client::broadcastTf(vpHomogeneousMatrix M, std::string parent, std::string child){

    geometry_msgs::Transform tf_msg;
    tf_msg = visp_bridge::toGeometryMsgsTransform(M);

    KDL::Frame frame;
    tf::transformMsgToKDL(tf_msg,frame);

	tf::Transform tf;
	tf::transformKDLToTF(frame,tf);

	br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), parent, child));
}

void Client::broadcastTf(KDL::Frame frame, std::string parent, std::string child){

	tf::Transform tf;
	tf::transformKDLToTF(frame,tf);

	br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), parent, child));
}

KDL::Frame Client::toKDLFrame(vpHomogeneousMatrix M){

    geometry_msgs::Transform tf_msg;
    tf_msg = visp_bridge::toGeometryMsgsTransform(M);

    KDL::Frame frame;
    tf::transformMsgToKDL(tf_msg,frame);

    return frame;
}


void Client::initAndSimulate_CameraToRobot(double pause_time, double noise_ampl)
{
  ROS_INFO("Camera fixed to the robot - Waiting for topics...");
  ros::Duration(1.).sleep();
  while(!reset_service_.call(reset_comm)){
    if(!ros::ok()) return;
    ros::Duration(1).sleep();
  }


  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  const int N = 9;
  ROS_INFO_STREAM("Number of stations: " << N <<std::endl);
  // Input: six couple of poses used as input in the calibration proces

  // Define the world frame
  vpHomogeneousMatrix I;

  // Define the end-effector to camera transform
  eTc.p.x(0.12);
  eTc.p.z(0.07);
  eTc.M = KDL::Rotation::RPY(vpMath::rad(0),vpMath::rad(0),vpMath::rad(45));

  geometry_msgs::Transform pose_e_c;
  tf::transformKDLToMsg(eTc,pose_e_c);
  ROS_INFO("1) GROUND TRUTH:");
  ROS_INFO_STREAM("End-effector to camera transformation: " <<std::endl<<pose_e_c<<std::endl);

  // Define the object location w.r.t. the world frame
  wTo.p.x(0.0);
  wTo.p.y(-0.1);

  // Define the robot location w.r.t the world frame
  double radius = 0.5;
  wTb.p.x(0.0);
  wTb.p.y(0.2 + radius);

  // Define the robot to object transform
  bTo = wTb.Inverse() * wTo;

  // 6 poses
  double max_noise = std::numeric_limits<double>::min();
  double min_noise = std::numeric_limits<double>::max();
  // initialize random seed
  srand (time(NULL));
  // define msg for desired robot poses
  visp_hand2eye_calibration::TransformArray desired_endeffector_poses_msg;
  for (int i = 0; i < N; i++)
  {

	oTc.p.x(radius*cos(i*M_PI/(N-1)));
	oTc.p.y(radius*sin(i*M_PI/(N-1)));
	oTc.p.z(radius);
	oTc.M = KDL::Rotation::RPY(0, -3*M_PI/4, i*M_PI/(N-1));
	oTc.M.DoRotZ(i*M_PI/((N-1)));
	cTo = oTc.Inverse();

	bTe = wTb.Inverse() * wTo * oTc * eTc.Inverse();

	double noise = 0.01*(((rand() % 100) - 50) /50.0);
	max_noise=std::max(max_noise,noise);
	min_noise=std::min(min_noise,noise);
	bTe.p.x( bTe.p.x() + noise);

	broadcastTf(I,"/world","/world_");
	broadcastTf(wTb,"/world","/base");
	broadcastTf(wTo,"/world","/object");
	broadcastTf(oTc,"/object","/camera1");
	broadcastTf(eTc.Inverse(),"/camera1","/end-effector1");
	broadcastTf(bTe,"/base","/end-effector1_measured");

    geometry_msgs::Transform pose_c_o;
    tf::transformKDLToMsg(cTo,pose_c_o);
    geometry_msgs::Transform pose_b_e;
    tf::transformKDLToMsg(bTe,pose_b_e);

    desired_endeffector_poses_msg.transforms.push_back(pose_b_e);
    transform_comm.request.camera_object.transforms.push_back(pose_c_o);
    transform_comm.request.world_effector.transforms.push_back(pose_b_e);

    ros::Duration(pause_time).sleep();

  }
  desired_endeffector_poses_.publish(desired_endeffector_poses_msg);
  ROS_INFO_STREAM("Noise min and max values : " << min_noise << " / " << max_noise << std::endl);

  ros::Duration(2).sleep();
}


void Client::initAndSimulate_CameraToWorld(double pause_time, double noise_ampl){

	  ROS_INFO("Camera fixed to the world - Waiting for topics...");
	  ros::Duration(1.).sleep();
	  while(!reset_service_.call(reset_comm)){
	    if(!ros::ok()) return;
	    ros::Duration(1).sleep();
	  }

	  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
	  const int N = 7;
	  ROS_INFO_STREAM("Number of stations: " << N <<std::endl);
	  // Input: six couple of poses used as input in the calibration proces

	  // Define the world frame
	  vpHomogeneousMatrix I;

	  // Define the end-effector to camera transform
	  eTm.p.x(0.015);
	  eTm.p.y(0.0);
	  eTm.p.z(0.03);
	  eTm.M = KDL::Rotation::RPY(vpMath::rad(0),vpMath::rad(0),vpMath::rad(-135));

	  geometry_msgs::Transform pose_e_m;
	  tf::transformKDLToMsg(eTm,pose_e_m);
	  ROS_INFO("1) GROUND TRUTH:");
	  ROS_INFO_STREAM("End-effector to marker transformation: " <<std::endl<<pose_e_m<<std::endl);

	  // Define the camera location w.r.t. the world frame
	  wTc.p.x(0.0);
	  wTc.p.y(-0.5);
	  wTc.p.z(0.5);
	  wTc.M.DoRotZ(M_PI/2);
	  wTc.M.DoRotY(M_PI/4);

	  // Define the robot location w.r.t the world frame
	  wTb.p.x(0.0);
	  wTb.p.y(0.0);

	  // Define the robot to camera transform
	  bTc = wTb.Inverse() * wTc;

	  // 6 poses
	  double max_noise = std::numeric_limits<double>::min();
	  double min_noise = std::numeric_limits<double>::max();
	  double max_noise_angular = std::numeric_limits<double>::min();
	  double min_noise_angular = std::numeric_limits<double>::max();
	  // initialize random seed
	  srand (time(NULL));
	  // define msg for desired robot poses
	  visp_hand2eye_calibration::TransformArray desired_endeffector_poses_msg;
	  for (int i = 0; i < N; i++)
	  {
		if (i==0){
			wTm.p.x(0.25);
			wTm.p.y(-0.5);
			wTm.p.z(0.3);
			wTm.M = KDL::Rotation::RPY( 0,0,-M_PI);
		}
		else if (i==1){
			wTm.p.y(0.1);
			wTm.p.z(0.4);
			wTm.M.DoRotZ(M_PI/4);
			wTm.M.DoRotY(M_PI/4);
		}
		else if (i==2){
			wTm.p.x(0.0);
			wTm.p.y(-0.1);
			wTm.p.z(0.3);
			wTm.M = KDL::Rotation::RPY( 0,0,-M_PI);
		}
		else if (i==3){
			wTm.p.z(0.4);
			wTm.p.y(-0.6);
			wTm.M = KDL::Rotation::RPY( 0,0,-M_PI);
			wTm.M.DoRotZ(M_PI/4);
			wTm.M.DoRotY(M_PI/4);
		}
		else if (i==4){
			wTm.p.z(0.4);
			wTm.p.y(-0.6);
			wTm.M = KDL::Rotation::RPY( 0,0,-M_PI);
			wTm.M.DoRotZ(-M_PI/4);
			wTm.M.DoRotY(-M_PI/4);
		}
		else if (i==5){
			wTm.p.x(-0.25);
			wTm.p.y(-0.5);
			wTm.p.z(0.3);
			wTm.M = KDL::Rotation::RPY( 0,0,-M_PI);
		}
		else if (i==6){
			wTm.p.y(-0.1);
			wTm.p.z(0.2);
     		wTm.M.DoRotZ(-M_PI/4);
			wTm.M.DoRotY(-M_PI/4);
		}




		cTm = wTc.Inverse() * wTm;
		mTc = cTm.Inverse();
		bTe = wTb.Inverse() * wTc * cTm * eTm.Inverse();

		// Translation noise
		double noise_linear = noise_ampl*(((rand() % 100) - 50) /50.0);
		max_noise=std::max(max_noise,noise_linear);
		min_noise=std::min(min_noise,noise_linear);
		bTe.p.x( bTe.p.x() + noise_linear);
		bTe.p.y( bTe.p.y() + noise_linear);

		// Rotational noise
		double noise_angular = M_PI*(100*noise_ampl*(((rand() % 100) - 50) /50.0))/180;
		max_noise_angular=std::max(max_noise_angular,noise_angular);
		min_noise_angular=std::min(min_noise_angular,noise_angular);
		bTe.M.DoRotX(noise_angular);


		broadcastTf(I,"/world","/world_");
		broadcastTf(wTb,"/world","/base");
		broadcastTf(wTc,"/world","/camera2");
		broadcastTf(cTm,"/camera2","/marker");
		broadcastTf(eTm.Inverse(),"/marker","/end-effector2");
		broadcastTf(bTe,"/base","/end-effector2_measured");

	    geometry_msgs::Transform pose_m_c;
	    tf::transformKDLToMsg(mTc,pose_m_c);
	    geometry_msgs::Transform pose_b_e;
	    tf::transformKDLToMsg(bTe,pose_b_e);

	    desired_endeffector_poses_msg.transforms.push_back(pose_b_e);
	    transform_comm.request.camera_object.transforms.push_back(pose_m_c);
	    transform_comm.request.world_effector.transforms.push_back(pose_b_e);

	    ros::Duration(pause_time).sleep();

	  }
	  desired_endeffector_poses_.publish(desired_endeffector_poses_msg);
	  ROS_INFO_STREAM("Translational noise - limits (m) : " << min_noise << " / " << max_noise << std::endl);
	  ROS_INFO_STREAM("Rotational noise - limits (degrees) : " << 180*min_noise_angular/M_PI << " / " << 180*max_noise_angular/M_PI << std::endl);



}

void Client::sendComputingRequest()
{

  ROS_INFO("Computing request send to calibration node:");
  if (compute_transform_service_.call(transform_comm))
  {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << transform_comm.response.effector_camera);
    KDL::Frame result;
	tf::transformMsgToKDL(transform_comm.response.effector_camera,result);
	KDL::Vector translation_error_vector = (eTm.p - result.p);
    ROS_INFO_STREAM("Translation error: " << translation_error_vector.Norm());
    KDL::Vector axis;
    KDL::Vector axis_result;
    double angle, angle_result;
    angle = eTm.M.GetRotAngle(axis);
    angle_result = result.M.GetRotAngle(axis_result);
    double axis_diff = KDL::dot(axis,axis_result);
    double angle_diff = angle - angle_result;
    ROS_INFO_STREAM("Rotation angle error: " << 180*angle_diff/M_PI << " degrees" <<std::endl);
    ROS_INFO_STREAM("Rotation axis error: " << 180*acos(axis_diff)/M_PI << " degrees" <<std::endl);

  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
