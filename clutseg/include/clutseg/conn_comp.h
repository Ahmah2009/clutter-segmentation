/*
 * Author: Julius Adorf
 */

#ifndef _CONN_COMP_H_
#define _CONN_COMP_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
#include "clutseg/gcc_diagnostic_enable.h"

/**
 * \brief Finds the largest connected components of white pixels in a binary image,
 * and fills all smaller connected components with the black background color.
 *
 * See method cv::findContours for what type of images are accepted.
 */
void largestConnectedComponent(cv::Mat& img);

#endif
