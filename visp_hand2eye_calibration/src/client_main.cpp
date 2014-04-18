/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
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
 * Entry point for client node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file calibrator_main.cpp
  \brief Entry point for client node
*/

#include "client.h"
#include "ros/ros.h"

int main(int argc,char**argv){
  ros::init(argc, argv, "client");

  visp_hand2eye_calibration::Client ct;

  // The first argument defines the scenario: camera fixed to robot or to world
  unsigned int mode = 2;
  if (argc > 1)
	  mode = atoi(argv[1]);

  // The second argument defines the pause_time in between displaying to calibration poses
  double pause_time = 0.2;
  if (argc > 2)
	  pause_time = atof(argv[2]);

  // The third argument defines the 'dimension' of the robot motion
  double radius = 0.3;
  if (argc > 3)
	  radius = atof(argv[3]);

  ROS_INFO_STREAM("MODE: " << mode << std::endl);
  if (mode == 1){
	  ct.initAndSimulate_CameraToRobot(pause_time,radius);
  }else{
	  ct.initAndSimulate_CameraToWorld(pause_time,radius);
  }

  ct.sendComputingRequest();
  return 0 ;
}
