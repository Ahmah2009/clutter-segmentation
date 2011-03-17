/*
 * Author: Julius Adorf 
 */

#include <iostream>

using namespace std;

/** \brief Prepares training data for the feature detection and extraction
 * stage in the training pipeline of ROS package tod_training.
 */
int main(int argc, char *argv[])
{
    // Input:
    // - training base directory
    // - directory with all training data for one specific object

    // TODO: load options from command line using boost::program_options
    // Options opts;
    // options(argc, argv, opts);

    if (argc != 3)
    {
        cerr << "Usage: preparetraining <bags-dir> <base-dir>" << endl;
        return 1;
    }

    // This directory contains all the data from TUM/IAS semantic3d database.
    string obj_dir(argv[1]);
    // This directory hosts the training base that is to be prepared. 
    string train_dir(argv[2]);

    cout << obj_dir << endl;
    cout << train_dir << endl;

    

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
