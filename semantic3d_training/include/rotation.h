/**
 * Author: Julius Adorf
 */

#include <cv.h>

// TODO: are cv::Mat objects automatically placed on the heap, or can
// we just return them as references like in Java??? The operators on
// matrices might give clues.

cv::Mat generateXRotationMatrix(double angle);

cv::Mat generateYRotationMatrix(double angle);

cv::Mat generateZRotationMatrix(double angle);

