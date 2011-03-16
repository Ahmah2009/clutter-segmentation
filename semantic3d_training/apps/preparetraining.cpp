/*
 * preparetraining.cpp
 *
 * Author: Julius Adorf 
 */


/** \brief Prepares training data for the feature detection and extraction
 * stage in the training pipeline of ROS package tod_training.
 */
int main(int argc, char *argv[])
{
    // Input:
    // - training base directory
    // - directory with all training data for one specific object

    // list images
    // for v in views do
    //     p = estimate_pose(v)
    //     m = create_mask(v)
    //     p.serialize()
    //     m.serialize()
    // done

    // Output:
    // - black and white mask representing the region of interest for each training image
    // - pose estimation for each training image
}
