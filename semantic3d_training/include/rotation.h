/**
 * Author: Julius Adorf
 */

#include <cv.h>

// TODO: are cv::Mat objects automatically placed on the heap, or can
// we just return them as references like in Java??? The operators on
// matrices might give clues.

void populateXRotationMatrix(cv::Mat rot, double angle);

void populateYRotationMatrix(cv::Mat rot, double angle);

void populateZRotationMatrix(cv::Mat rot, double angle);

