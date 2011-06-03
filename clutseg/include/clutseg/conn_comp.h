/**
 * Author: Julius Adorf
 */

#ifndef _CONN_COMP_H_
#define _CONN_COMP_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
#include "clutseg/gcc_diagnostic_enable.h"

/** Finds the largest connected components and fills all other connected
 * components with background color. Background is black and foreground is
 * white. See method cv::findContours for what type of images are accepted. */
void largestConnectedComponent(cv::Mat& img);

#endif
