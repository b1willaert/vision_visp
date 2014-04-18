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
  \file client.h
  \brief Client node calling a quick compute service, a compute service and 2 publishing to world_effector_topic and camera_object_topic.
*/

#ifndef __visp_hand2eye_calibration_CLIENT_H__
#define __visp_hand2eye_calibration_CLIENT_H__
#include "ros/ros.h"
#include "geometry_msgs/Transform.h"
#include "visp_hand2eye_calibration/compute_effector_camera.h" 
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h" 
#include "visp_hand2eye_calibration/reset.h" 

#include <tf/transform_broadcaster.h>
#include <visp/vpExponentialMap.h>

#include <kdl_conversions/kdl_msg.h>


namespace visp_hand2eye_calibration{ 
  class Client{
  private:
    ros::NodeHandle n_;
    ros::Publisher desired_endeffector_poses_;

    ros::ServiceClient reset_service_;
    ros::ServiceClient compute_transform_service_;

    visp_hand2eye_calibration::reset reset_comm;
    visp_hand2eye_calibration::compute_effector_camera_quick transform_comm;
    
	tf::TransformBroadcaster br;

  public:
    Client();

    void broadcastTf(vpHomogeneousMatrix m, std::string parent, std::string child);
    void broadcastTf(KDL::Frame frame, std::string parent, std::string child);
    KDL::Frame toKDLFrame(vpHomogeneousMatrix M);

    void initAndSimulate_CameraToRobot(double pause_time, double radius);
    void initAndSimulate_CameraToWorld(double pause_time, double radius);

    void sendComputingRequest();
 };
}
#endif
