/**
 * Author: Julius Adorf
 *
 * This application shall find good parameters for tod_training, tod_detecting
 * and clutseg. Basically, it does so by factorial design. Since the parameter
 * space is extremely large, the factors have to be carefully selected. Also,
 * results must be meticoulusly recorded.
 *
 * This application assumes that training set and test set are fixed. Results
 * are stored in a sqlite3 database.
 */

#include <clutseg/clutseg.h>

struct ClutsegParams {

    // bundles whatever parameters are used in ClutSegmenter

};

int main(int argc, char **argv) {
    // Use different feature extraction parameters on the training set. Loading these
    // features is a rather costly operation and shall therefore be made in the outer 
    // loop only. It is not too expensive though, and therefore we can generate these
    // features on the fly using existing scripts.
    vector<FeatureExtractionParams> trainingParams;
    // The parameter configurations are retrieved from the database. There is a
    // marker whether a configuration has already been evaluated. Thus, the
    // process can be interrupted without breaking the work of the last few
    // hours. Also, the parameter configuration and the generation of new
    // parameter sets is somewhat separated.
    for (size_t i = 0; i < trainingParams.size(); i++) {
        // Extract training features
        // Load training base

        // IDEA: cache extracted features from images, saves time

        vector<ClutsegParams> recogParams;
        for (size_t j = 0; j < recogParams.size(); j++) {
            // it's best to randomly choose a couple
            // of images out of a large testing set such to prevent having to
            // test too many configurations yet to get a too large bias.  in
            // favor of certain parameter configurations. So if several
            // parameter configurations are tested that are close to each other
            // and one of the parameter configurations shows extreme results
            // compared to the others and we assume a somewhat well-behaving
            // problem, then we can notice such an outlier.
            testSubset = randomSubset(testingSet);
            for (size_t k = 0; k < testSubset.size(); k++) {
                // run on a single image
                // compare with ground truth
            }
            // enter parameters and results into table
            // generate associated filesystem
            // timing
            // save time and date
            // save git commit
        }
    }

    return 0;
}

