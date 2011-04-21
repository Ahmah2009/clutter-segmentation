/*
 * Author: Julius Adorf
 */

#include "guess_util.h"
#include <boost/foreach.hpp>

namespace clutseg {

    Mat drawAllMatches(Mat canvas, const TrainingBase & base,
                            const Ptr<Matcher> matcher, const Mat& testImage,
                            const KeypointVector & testKeypoints, const string & baseDirectory) {
      namedWindow("matches", CV_WINDOW_KEEPRATIO);
      vector<Mat> match_images;
      int scaled_width = 1000;
      int scaled_height = 0;

      // Build the individual matches
      for (size_t objectInd = 0; objectInd < base.size(); objectInd++)
      {
        for (size_t imageInd = 0; imageInd < base.getObject(objectInd)->observations.size(); imageInd++)
        {
          tod::Matches imageMatches;
          matcher->getImageMatches(objectInd, imageInd, imageMatches);
          if (imageMatches.size() > 7)
          {
            Features3d f3d = base.getObject(objectInd)->observations[imageInd];
            Features2d f2d = f3d.features();
            if (f2d.image.empty())
            {
              string filename = baseDirectory + "/" + base.getObject(objectInd)->name + "/" + f2d.image_name;
              f2d.image = imread(filename, 0);
            }
            Mat matchesView;
            drawMatches(testImage, testKeypoints, f2d.image,
                            base.getObject(objectInd)->observations[imageInd].features().keypoints, imageMatches,
                            matchesView, Scalar(255, 0, 0), Scalar(0, 0, 255));
            // Resize the individual image
            Size smaller_size(scaled_width, (matchesView.rows * scaled_width) / matchesView.cols);
            Mat resize_match;
            resize(matchesView, resize_match, smaller_size);
            // Keep track of the max height of each image
            scaled_height = std::max(scaled_height, smaller_size.height);

            // Add the image to the big_image
            match_images.push_back(resize_match);
          }
        }
      }
      // Display a big image with all the correspondences
      unsigned int big_x = 0, big_y = 0;
      unsigned int n_match_image = 0;
      while (n_match_image < match_images.size())
      {
        if (1.5 * big_x < big_y)
        {
          big_x += scaled_width;
          n_match_image += big_y / scaled_height;
        }
        else
        {
          big_y += scaled_height;
          n_match_image += big_x / scaled_width;
        }
      }
      Mat_<Vec3b> big_image = Mat_<Vec3b>::zeros(big_y, big_x);
      int y = 0, x = 0;
      BOOST_FOREACH(const Mat & match_image, match_images)
            {
              Mat_<Vec3b> sub_image = big_image(Range(y, y + scaled_height), Range(x, x + scaled_width));
              match_image.copyTo(sub_image);
              x += scaled_width;
              if (x >= big_image.cols)
              {
                x = 0;
                y += scaled_height;
              }
            }
      if (!big_image.empty()) {
            return big_image;
        } else {
            cout << "[WARNING] Big image is empty!" << endl;
           return big_image;
        }
    }

    void drawInliers(Mat & outImg, const Guess & guess, const Mat & testImage) {
        vector<KeyPoint> kpts;
        for (size_t i = 0; i < guess.inliers.size(); i++) {
            Point2f kp = guess.image_points_[guess.inliers[i]];
            kpts.push_back(KeyPoint(kp, 0));
        }
        drawKeypoints(testImage, kpts, outImg, Scalar(0, 255, 0));    
    }

}

