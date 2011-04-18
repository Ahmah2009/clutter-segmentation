/**
 * Author: Julius Adorf
 */

#include <cv.h>

using namespace cv;

namespace clutseg {

    /** Generates a rotation matrix R such that Rv is the vector v rotated about
     * the x-axis. */
    Mat generateXRotationMatrix(double angle);

    /** Generates a rotation matrix R such that Rv is the vector v rotated about
     * the y-axis. */
    Mat generateYRotationMatrix(double angle);

    /** Generates a rotation matrix R such that Rv is the vector v rotated about
     * the z-axis. */
    Mat generateZRotationMatrix(double angle);

}

