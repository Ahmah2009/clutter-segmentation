/*
 * Author: Julius Adorf
 */

#include "clutseg.h"

#include <boost/foreach.hpp>

namespace clutseg {
        
    ClutSegmenter::ClutSegmenter() {}

    ClutSegmenter::ClutSegmenter(Options & _opts, TrainingBase & _base) : opts(_opts), base(_base) {}

    bool ClutSegmenter::recognize(const Mat & queryImage, const PointCloudT & queryCloud, Guess & resultingGuess, PointCloudT & inliersCloud) {
        // Initialize matcher and recognizer. This must be done prior to every
        // query,
        Ptr<FeatureExtractor> extractor = FeatureExtractor::create(opts.params.feParams);
        Ptr<Matcher> rtMatcher = Matcher::create(opts.params.matcherParams);
        rtMatcher->add(base);
        cv::Ptr<Recognizer> recognizer = new KinectRecognizer(&base, rtMatcher,
                                &opts.params.guessParams, 0 /* verbose */, opts.baseDirectory);

        Features2d test;
        test.image = queryImage;
        vector<Guess> guesses;
        extractor->detectAndExtract(test);
        recognizer->match(test, guesses);
        if (guesses.empty()) {
            return false;
        } else {
            // Select guess with most inliers. We could also use another scheme
            // to determine the best guess, for example the guess where the
            // bounding rect.  of its inliers has the longest diagonal.
            int max_i = 0;
            size_t max_v = 0;
            for (size_t i = 0; i < guesses.size(); i++) {
                if (guesses[i].inliers.size() > max_v) {
                    max_v = guesses[i].inliers.size();
                    max_i = i;
                }
            }
            resultingGuess = guesses[max_i];

            // Find inlier points in query cloud. Note that we do not use
            // camera information here. Instead we assume that indices in query
            // cloud correspond to the 2d indices. This makes sense especially
            // when both 2d and 3d data has been taken from a Kinect camera.
            // Nevertheless, this assumption has to be verified. Also, it seems
            // that in tod_training, tod::PCLToPoints is solving the same task.
            // thing. Yet I think either my code has a bug or there code which
            // maps 2d to 3d points. This has to be investigated. See line 117
            // of clouds.h in tod_training. Why is the x-scale factor used for
            // y-scaling?
            // TODO: is this code snippet correct?
            float scaleW = float(queryCloud.width) / queryImage.cols;
            float scaleH = float(queryCloud.height) / queryImage.rows;
            BOOST_FOREACH(int idx, resultingGuess.inliers) {
                int u = int(resultingGuess.image_points_[idx].x * scaleW);
                int v = int(resultingGuess.image_points_[idx].y * scaleH);
                if (u < queryCloud.width && v < queryCloud.height) {
                    inliersCloud.push_back(queryCloud(u, v));
                } else {
                    cerr << "WARNING: cannot find 3d point for inlier, outside of point cloud" << endl; 
                }
            }
            return true;
        }
    }
}

