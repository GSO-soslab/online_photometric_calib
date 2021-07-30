//
//  OpencvIncludes.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

/**
 * Defines all kinds of standard includes that are used in most of the classes
 */

#ifndef OnlineCalibration_StandardIncludes_h
#define OnlineCalibration_StandardIncludes_h

/**
 * Opencv includes
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Standard library includes
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <atomic>
#include <thread>
// #include <pthread.h>
// #include <fstream>

struct OptimizationParam {
  int image_width;
  int image_height;
  int visualize_cnt;
  int tracker_patch_size;
  int nr_pyramid_levels; 
  int nr_active_features;
  int nr_images_rapid_exp;
  int safe_zone_size;
  int nr_active_frames;
  int keyframe_spacing;
  int min_keyframes_valid;  
};

#endif
