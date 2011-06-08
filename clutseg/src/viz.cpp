/*
 * Author: Julius Adorf
 */

#include "clutseg/viz.h"

#include "clutseg/pose.h"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace std;
using namespace tod;
using namespace opencv_candidate;

namespace clutseg {

    vector<Scalar> predefColors;
    
    void initPredefColors() {
        if (predefColors.empty()) {
            predefColors.push_back(Scalar(0, 0, 255));
            predefColors.push_back(Scalar(0, 255, 0));
            predefColors.push_back(Scalar(255, 0, 0));
            predefColors.push_back(Scalar(255, 0, 255));
            predefColors.push_back(Scalar(0, 255, 255));
            predefColors.push_back(Scalar(255, 255, 0));
            predefColors.push_back(Scalar(125, 0, 204));
            predefColors.push_back(Scalar(125, 204, 0));
            predefColors.push_back(Scalar(204, 0, 125));
            predefColors.push_back(Scalar(204, 125, 0));
            predefColors.push_back(Scalar(0, 204, 125));
            predefColors.push_back(Scalar(0, 125, 204));
        }
    }

    void drawKeypoints(Mat & canvas, const vector<KeyPoint> & keypoints,
                        const Scalar & color) { 
        // Use OpenCV's drawKeypoints method DrawMatchesFlags::DRAW_OVER_OUTIMG
        // specifies that keypoints are simply drawn onto existing content of
        // the output canvas. The first image parameter of drawKeypoints is
        // then not used by drawKeypoints. See also
        // https://code.ros.org/svn/opencv/trunk/opencv/modules/features2d/src/draw.cpp
        cv::drawKeypoints(canvas, keypoints, canvas, color,
                            DrawMatchesFlags::DRAW_OVER_OUTIMG);
    }

    void drawInliers(Mat & canvas, const Guess & guess, const Scalar & color) {
        vector<KeyPoint> kpts;
        for (size_t i = 0; i < guess.inliers.size(); i++) {
            Point2f kp = guess.image_points_[guess.inliers[i]];
            kpts.push_back(KeyPoint(kp, 0));
        }
        clutseg::drawKeypoints(canvas, kpts, color);
    }

    void drawPose(Mat & canvas, const PoseRT & pose, const Camera & camera,
                const Scalar & colorX, const Scalar & colorY, const Scalar & colorZ,
                const string & labelX, const string & labelY, const string & labelZ) {
        // see fiducial::PoseDrawer(canvas, camera.K, pose);
        // most of the code has been copied from that function
        Point3f z(0, 0, 0.25);
        Point3f x(0.25, 0, 0);
        Point3f y(0, 0.25, 0);
        Point3f o(0, 0, 0);
        vector<Point3f> op(4);
        op[1] = x, op[2] = y, op[3] = z, op[0] = o;
        vector<Point2f> ip;
        projectPoints(Mat(op), pose.rvec, pose.tvec, camera.K, camera.D, ip);

        vector<Scalar> c(4); //colors
        c[0] = Scalar(255, 255, 255);
        c[1] = colorX; //Scalar(255, 0, 0);//x
        c[2] = colorY; //Scalar(0, 255, 0);//y
        c[3] = colorZ; //Scalar(0, 0, 255);//z
        line(canvas, ip[0], ip[1], c[1]);
        line(canvas, ip[0], ip[2], c[2]);
        line(canvas, ip[0], ip[3], c[3]);
        /* see question on answers.ros.org
        string scaleText = "scale 0.25 meters";
        int baseline = 0;
        Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
        rectangle(canvas, Point(10, 30 + 5), Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
        putText(canvas, scaleText, Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
        */
        putText(canvas, labelZ, ip[3], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[3], 1, CV_AA, false);
        putText(canvas, labelY, ip[2], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[2], 1, CV_AA, false);
        putText(canvas, labelX, ip[1], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[1], 1, CV_AA, false);
    }

    void drawGuess(Mat & canvas, const Guess & guess, const Camera & camera, const PoseRT ground_pose) {
        vector<Guess> guesses(1, guess);
        vector<PoseRT> ground_poses(1, ground_pose);
        drawGuesses(canvas, guesses, camera, ground_poses);
    }

    void drawGuesses(Mat & canvas, const vector<Guess> & guesses,
                        const Camera & camera, const vector<PoseRT> & ground_poses) { srand(time(NULL));
        initPredefColors();
        vector<Scalar> colors;
        colors.assign(predefColors.begin(), predefColors.end());
        for (size_t i = predefColors.size(); i < guesses.size(); i++) {
            colors.push_back(Scalar(
                50 + rand() % 206,
                50 + rand() % 206,
                50 + rand() % 206));
        }
        // Draw inliers first
        for (size_t i = 0; i < guesses.size(); i++) {
            drawInliers(canvas, guesses[i], colors[i]);
        }
        // Draw ground poses
        for (size_t i = 0; i < ground_poses.size(); i++) {
            if (ground_poses[i].estimated) {
                drawPose(canvas, ground_poses[i], camera,
                    Scalar(20, 20, 20), Scalar(125, 125, 125), Scalar(235, 235, 235),
                    "ground.x", "ground.y", "ground.z");
            }
        }
        // Draw guessed poses
        for (size_t i = 0; i < guesses.size(); i++) {
            drawPose(canvas, guesses[i].aligned_pose(), camera, colors[i], colors[i], colors[i]);
        }
        
        // Draw labels
        for (size_t i = 0; i < guesses.size(); i++) {
            drawLabelAtOrigin(canvas, guesses[i].aligned_pose(), camera,
                str(boost::format("%s (%d/%d)") % guesses[i].getObject()->name %
                guesses[i].inliers.size() % guesses[i].image_points_.size()), colors[i]);
        }
    }

    void drawGuesses(cv::Mat & canvas, const std::vector<tod::Guess> & guesses,
                        const opencv_candidate::Camera & camera) {
        vector<PoseRT> dummy;
        drawGuesses(canvas, guesses, camera, dummy);
    }

    void drawLabelAtOrigin(Mat & canvas, const PoseRT & pose, const Camera & camera, const string & label, const Scalar & color) {
        Point topleft = projectOrigin(pose, camera); 
        vector<string> legend;
        legend.push_back(label);
        drawText(canvas, legend, topleft + Point(10, 10), FONT_HERSHEY_SIMPLEX, 1.2, 2, color);
    }

    Rect drawText(Mat & outImg, const vector<string> & lines,
                        const Point & topleft, int fontFace, double fontScale, int thickness,
                        const Scalar & color) {
        int baseline = 0;
        // bottom right corner, is "pushed" down and right as appropriate
        Point br = topleft;
        // bottom left corner, is "pushed" down as appropriate
        Point bl = topleft;
        for (size_t i = 0; i < lines.size(); i++) {
            Size sz = getTextSize(lines[i], fontFace, fontScale, thickness, &baseline);
            if (i == 0) {
                bl.y += sz.height;
            } else {
                bl.y += 1.7*sz.height;
            }
            putText(outImg, lines[i], bl, fontFace, fontScale, color, thickness, CV_AA, false);
            br.y = bl.y;
            if (bl.x + sz.width > br.x) {
                br.x = bl.x + sz.width;
            }
        }
        return Rect(topleft, br);
    }

    void drawAllMatches(Mat & canvas, const TrainingBase & base,
                            const Ptr<Matcher> matcher, const Mat& testImage,
                            const KeypointVector & testKeypoints, const string & baseDirectory) {
      // This method has been copied from tod_detecting, and has been changed not
      // to display the image but pass it to the caller.
      vector<Mat> match_images;
      int scaled_width = 1000;
      int scaled_height = 0;
      size_t match_threshold = 7;

      // Build the individual matches
      for (size_t objectInd = 0; objectInd < base.size(); objectInd++)
      {
        for (size_t imageInd = 0; imageInd < base.getObject(objectInd)->observations.size(); imageInd++)
        {
          tod::Matches imageMatches;
          matcher->getImageMatches(objectInd, imageInd, imageMatches);
          if (imageMatches.size() > match_threshold)
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
      //Mat_<Vec3b> big_image = Mat_<Vec3b>::zeros(big_y, big_x);
      canvas.create(big_y, big_x, CV_8UC3);
      int y = 0, x = 0;
      BOOST_FOREACH(const Mat & match_image, match_images)
            {
              Mat_<Vec3b> sub_image = canvas(Range(y, y + scaled_height), Range(x, x + scaled_width));
              match_image.copyTo(sub_image);
              x += scaled_width;
              if (x >= canvas.cols)
              {
                x = 0;
                y += scaled_height;
              }
            }
      if (canvas.empty()) {
            canvas.create(400, 400, CV_8UC3);
            vector<string> info;
            info.push_back(str(boost::format("No training image having more than %d matches") % match_threshold));
            drawText(canvas, info, Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.7, 2, Scalar::all(255));
            cout << "[WARNING] Big image is empty!" << endl;
      }
    }

    void drawGroundTruth(Mat & canvas, const LabelSet & groundTruth,
                            const Camera & camera) {
        BOOST_FOREACH(const Label & np, groundTruth.labels) {
            drawPose(canvas, np.pose, camera,
                    Scalar(20, 20, 20), Scalar(125, 125, 125), Scalar(235, 235, 235),
                    np.name + ".x", np.name + ".y", np.name + ".z");
        }
    }
             
}

