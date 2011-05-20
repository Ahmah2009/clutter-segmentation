/*
 * blackbox_recognizer.cpp
 */

/* This recognizer takes a test configuration file that lists a number of test
 * images and the objects that can be found on each test image. It prints out
 * the image name, the actual objects on this image and those object that have
 * been predicted to be on the image. The advantage of such batch-processing is
 * speed because the training base has not to be reloaded each time.
 *
 * Originally derived from folder_recognizer.cpp in tod_detecting/apps. The
 * folder_recognizer.cpp in tod_detecting rev. 50320 is broken for several
 * reasons.
 * - it does only handle png files
 * - it interprets files that contain .png in their filename as images
 * - it inscrupulously tries to recognize objects on each image it can find
 *   within a directory, but does not attempt to recurse.
 * - it produces OpenCV errors
 * - it's command-line description is out of sync
 * - it generates spam sample.config.yaml files
 * - it is spamming standard output
 * - ...
 * 
 * This implementation fixes some of the issues. Also, it tries to collect as
 * much information about the performance and output of the classifier on a
 * given testing set.
 */

#include "clutseg/ground.h"
#include "clutseg/pose.h"
#include "clutseg/options.h"
#include "clutseg/viz.h"

#include <tod/training/stats.h>
#include <tod/detecting/Loader.h>
#include <tod/detecting/Recognizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_candidate/PoseRT.h>
#include <iostream>
#include <fstream>
#include <set>
#include <string>
#include <time.h>

#define foreach BOOST_FOREACH

using namespace cv;
using namespace opencv_candidate;
using namespace tod;
using namespace std;
using namespace boost;
using namespace clutseg;

bool readImage(Features2d & test, const string & path) {
    cout << boost::format("Reading <%s>") % path << endl;
    test.image = imread(path, 0);
    if (test.image.empty()) {
        cout << "Cannot read test image" << endl;
        return false;
    }
    return true;
}

void storeMatchDrawing(const string & fname, const Guess & guess, const string & baseDirectory, int flags) {
    Mat canvas;
    guess.draw(canvas, flags, baseDirectory);
    imwrite(fname, canvas);
}

void storeAllMatchesDrawing(const string & fname, const TrainingBase & base,
        const Ptr<Matcher> & rtMatcher, const Features2d & test, const string & baseDirectory) {
    Mat canvas;
    drawAllMatches(canvas, base, rtMatcher, test.image, test.keypoints, baseDirectory);
    imwrite(fname, canvas);
}

void storeQueryKeypointsDrawing(const string & fname, const Features2d & test) {
    Mat canvas;
    cvtColor(test.image, canvas, CV_GRAY2BGR);
    clutseg::drawKeypoints(canvas, test.keypoints);
    imwrite(fname, canvas);
}

void storeAlignedPoints(const string & fname, const Guess & guess) {
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    BOOST_FOREACH(const Point3f & p, guess.aligned_points_) {
        aligned_cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    pcl::io::savePCDFileASCII(fname, aligned_cloud);
}

void storeInliersDrawing(const string & fname, const Guess & guess, const Mat & testImage) {
    Mat canvas;
    cvtColor(testImage, canvas, CV_GRAY2BGR);
    drawInliers(canvas, guess);
    imwrite(fname, canvas);
}

void storeGuessDrawing(const string & fname, const Guess & guess, const Mat & testImage, const Camera & camera, const PoseRT & ground_posert = PoseRT()) {
    Mat canvas = testImage.clone();
    cvtColor(testImage, canvas, CV_GRAY2BGR);
    drawGuess(canvas, guess, camera, ground_posert);
    imwrite(fname, canvas);
}


int main(int argc, char *argv[])
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    tod::Loader loader(opts.baseDirectory);
    vector < cv::Ptr < TexturedObject > >objects;
    loader.readTexturedObjects(objects);

    if (objects.empty()) {
        cout << "Empty base\n" << endl;
        throw std::runtime_error("Empty training base.");
    }
    
    TestSetGroundTruth testdesc = loadTestSetGroundTruthWithoutPoses(boost::filesystem::path(opts.testdescFilename));

    bool write_store = (opts.storeDirectory != "");

    // If --storeDirectory is specified, all and as many results as possible
    // are to be written to that folder.
    if (write_store) {
        assert(filesystem::exists(opts.storeDirectory));
        opts.resultFilename = opts.storeDirectory + "/results.txt";
        opts.statsFilename = opts.storeDirectory + "/stats.txt";
        opts.rocFilename = opts.storeDirectory + "/roc.gnuplot";
        opts.tableFilename = opts.storeDirectory + "/table.csv";
    }

    bool write_result = (opts.resultFilename != "");
    bool write_table = (opts.tableFilename != "");
    bool write_stats = (opts.statsFilename != "");
    bool write_roc = (opts.rocFilename != "");

    ofstream result;
    ofstream roc;
    ofstream stats;
    ofstream table;

    if (write_result) {
        result.open(opts.resultFilename.c_str());
        result << "[predictions]" << endl;
    }
 
    if (write_roc) {
        roc.open(opts.rocFilename.c_str());
        roc << "set size .75,1" << endl;
        roc << "set size ratio 1" << endl;
        roc << "set xtics .1" << endl;
        roc << "set ytics .1" << endl;
        roc << "set grid" << endl;
        roc << "set xrange [0:1.025]" << endl;
        roc << "set yrange [0:1.025]" << endl;
        roc << "set key right bottom" << endl;
        roc << "set ylabel \"True Positive Rate\"" << endl;
        roc << "set xlabel \"False Positive Rate\" " << endl;
        roc << "set pointsize 2 " << endl;
        roc << "plot x with lines, '-' with points" << endl;
    }

    if (write_stats) {
        stats.open(opts.statsFilename.c_str());
        time_t today;
        time(&today);
        stats << boost::format("# date: %s") % today << endl;
        stats << boost::format("# training base: %s") % opts.baseDirectory << endl;
        stats << "[statistics]" << endl;
    }

    if (write_store) {
        filesystem::copy_file(opts.baseDirectory + "/features.config.yaml", opts.storeDirectory + "/features.config.yaml");
        filesystem::copy_file(opts.config, opts.storeDirectory + "/config.yaml");
        filesystem::copy_file(opts.testdescFilename, opts.storeDirectory + "/testdesc.txt");
    }

    if (write_table) {
        table.open(opts.tableFilename.c_str());
        table << boost::format("%-45s %-25s %-5s %-3s %-7s "
                           "%-10s %-10s %-10s %-10s %-10s %-10s "
                           "%-10s %-10s %-10s %-10s %-10s %-10s "
                           "%-10s %-10s") 
                            % "image" % "object" % "guess" % "hit" % "inliers"
                            % "guess_tx" % "guess_ty" % "guess_tz" % "guess_rx" % "guess_ry" % "guess_rz"
                            % "ground_tx" % "ground_ty" % "ground_tz" % "ground_rx" % "ground_ry" % "ground_rz" 
                            % "max_rerr_t" % "max_rerr_r" << endl;
    }
  
    int objectIndex = 1;

    // count hits, misses, error 1, error 2.  see Receiver operating
    // characteristics.  true/false positives/negatives.  A confusion matrix
    // has 4 dof and it's easiest to keep track of tp, fp and the column sums
    // to compute tn, fn later.

    int tp_acc = 0;
    int fp_acc = 0;
    int tn_acc = 0;
    int fn_acc = 0;
    int p_acc = 0;
    int n_acc = 0;

    assert(!objects.empty());
    const Camera & trainingCamera = objects[0]->observations[0].camera();

    for (TestSetGroundTruth::iterator it = testdesc.begin(); it != testdesc.end(); it++) {
        // Actually, the following lines of code could probably be moved out of
        // the loop again, and can be reused. I just want to make sure that
        // there is independence between test runs and that there is no way
        // that any result of previous test image runs will influence the
        // result of succeeding tests (silly keypoints problem). Yet, IMHO it's
        // highly unlikely that there is any dependence between the results.
        // In anyway, it's cheap to recreate those instances. Remark: this is due to a
        // bug in DynamicAdaptedFeatureAdaptor in OpenCV.
        TrainingBase base(objects);
        detector_stats detector_stats;
        Ptr < FeatureExtractor > extractor =
            FeatureExtractor::create(opts.params.feParams, detector_stats);

        cv::Ptr < Matcher > rtMatcher =
            Matcher::create(opts.params.matcherParams);
        rtMatcher->add(base);

        cv::Ptr < Recognizer > recognizer;
        if (opts.mode == TOD) {
            recognizer =
                new TODRecognizer(&base, rtMatcher, &opts.params.guessParams,
                                  opts.verbose, opts.baseDirectory,
                                  opts.params.clusterParams.maxDistance);
        } else if (opts.mode == KINECT) {
            recognizer =
                new KinectRecognizer(&base, rtMatcher, &opts.params.guessParams,
                                     opts.verbose, opts.baseDirectory);
        } else {
            cout << "Invalid mode option!" << endl;
            return 1;
        } // end of block that could probably be moved out of the loop 

        GroundTruth groundTruth = it->second; 
        // true positives
        int tp = 0;
        // false positives
        int fp = 0;
        // total positives
        int p = groundTruth.labels.size();

        string img_name = it->first;
        // In case the images are in subfolders of the test directory, we either
        // have to replicate that layout in the result folder or flatten the filename.
        // Decided for the latter approach since it simplifies browsing through the 
        // results with eog or other tools.
        string mapped_img_name(img_name);
        algorithm::replace_all(mapped_img_name, "/", "__");
        // Prefix identifying the test image
        string test_name = str(boost::format("%s/%s") % opts.storeDirectory % mapped_img_name);

        Features2d test;
        
        string path = opts.imageDirectory + "/" + img_name;
        if (!readImage(test, path)) {
            cerr << "Error: cannot read image" << endl;
            return -1;
        }

        extractor->detectAndExtract(test);

        vector<Guess> guesses;
        recognizer->match(test, guesses);

        storeAllMatchesDrawing(test_name + ".all.matches.png", base, rtMatcher, test, opts.baseDirectory);
        storeQueryKeypointsDrawing(test_name + ".keypoints.png", test);

        // The set of detected objects on the query image.
        set<string> found;
        // The number of guesses per detected object.
        map<string, int> guess_count;
        // The vector of ground poses, if one pose is not available or for
        // false alarms, this vector will have a pose with pose.estimated =
        // false.
        vector<PoseRT> ground_poses;
        foreach(const Guess & guess, guesses) {
            string name = guess.getObject()->name;
            Pose guess_pose = guess.aligned_pose();
            PoseRT guess_posert = poseToPoseRT(guess_pose);
            
            found.insert(name);
            guess_count[name] += 1;

            // Try whether there is ground-truth on the object pose in the test
            // images.  The pose is stored in files named
            // <image_name>.<name>.ground.pose.yaml, such as
            // image_00123.png.odwalla_lime.ground.pose.yaml or
            // image_00123.png.tide.ground.pose.yaml It is not possible to
            // specify different poses for items that appear more than one time
            // on the test image, but that leads to much more complex
            // questions.  The following code assumes that there is only one
            // instance of every training subject on a test image.
            PoseRT ground_posert;
            string ground_pose_path = str(boost::format("%s/%s.%s.ground.pose.yaml") % opts.imageDirectory % img_name % name);
            bool ground_pose_available = filesystem::exists(ground_pose_path);
            if (ground_pose_available) {
                readPose(ground_pose_path, ground_posert);
            }
            ground_poses.push_back(ground_posert);
 
            bool is_tp = groundTruth.onScene(name);
          
            //  drawGuess2(allGuessesImg, guess, test.image, trainingCamera, false, ground_posert);

            if (write_store) {
                // Prefix identifying a certain subject on a test image
                string test_subj_name = str(boost::format("%s.%s") % test_name % name);
                // Prefix identifying a certain guess for a specific subject on a test image
                string test_guess_name = str(boost::format("%s.%d") % test_subj_name % guess_count[name]);
                string guessed_pose_path = test_guess_name + ".guess.pose.yaml";
                string ground_pose_path = test_guess_name + ".ground.pose.yaml";
                writePose(guessed_pose_path, guess_posert);
                storeMatchDrawing(test_guess_name + ".projection.png", guess, opts.baseDirectory, 0);
                storeMatchDrawing(test_guess_name + ".merged.matches.png", guess, opts.baseDirectory, 1);
                storeGuessDrawing(test_guess_name + ".guess.png", guess, test.image, trainingCamera, ground_posert);
                // this is a cloud stored on a per-object basis, so it 
                // might be rewritten some times in this loop
                storeAlignedPoints(test_subj_name + ".pcd", guess);
                // TODO: store inliers as keypoint vector
                storeInliersDrawing(test_guess_name + ".inliers.png", guess, test.image);
            }

            if (write_table) {
                // there is a fuckup with mixing up doubles and floats
                // guessed pose
                double guess_tx = guess_posert.tvec.at<double>(0, 0);
                double guess_ty = guess_posert.tvec.at<double>(1, 0);
                double guess_tz = guess_posert.tvec.at<double>(2, 0);
                double guess_rx = guess_posert.rvec.at<double>(0, 0);
                double guess_ry = guess_posert.rvec.at<double>(1, 0);
                double guess_rz = guess_posert.rvec.at<double>(2, 0);
                // ground-truth pose
                double ground_tx = numeric_limits<double>::quiet_NaN();
                double ground_ty = numeric_limits<double>::quiet_NaN();
                double ground_tz = numeric_limits<double>::quiet_NaN();
                double ground_rx = numeric_limits<double>::quiet_NaN();
                double ground_ry = numeric_limits<double>::quiet_NaN();
                double ground_rz = numeric_limits<double>::quiet_NaN();
                double max_rerr_t = numeric_limits<double>::quiet_NaN();
                double max_rerr_r = numeric_limits<double>::quiet_NaN();
                if (ground_pose_available) {
                    ground_tx = ground_posert.tvec.at<double>(0, 0);
                    ground_ty = ground_posert.tvec.at<double>(1, 0);
                    ground_tz = ground_posert.tvec.at<double>(2, 0);
                    ground_rx = ground_posert.rvec.at<double>(0, 0);
                    ground_ry = ground_posert.rvec.at<double>(1, 0);
                    ground_rz = ground_posert.rvec.at<double>(2, 0);

                    // relative errors
                    double rerr_t[3];
                    double rerr_r[3];
                    rerr_t[0] = abs(guess_tx - ground_tx) / ground_tx;
                    rerr_t[1] = abs(guess_ty - ground_ty) / ground_ty;
                    rerr_t[2] = abs(guess_tz - ground_tz) / ground_tz;
                    rerr_r[0] = abs(guess_rx - ground_rx) / ground_rx;
                    rerr_r[1] = abs(guess_ry - ground_ry) / ground_ry;
                    rerr_r[2] = abs(guess_rz - ground_rz) / ground_rz;
                    max_rerr_t = *max_element(rerr_t, rerr_t + 3);
                    max_rerr_r = *max_element(rerr_r, rerr_r + 3);
                }

                table << boost::format("%-45s %-25s %5d %3d %7d "
                           "%10.7f %10.7f %10.7f %10.7f %10.7f %10.7f "
                           "%10.7f %10.7f %10.7f %10.7f %10.7f %10.7f "
                           "%10.7f %10.7f") 
                            % img_name % name % guess_count[name]  % (is_tp ? 1  : 0) % guess.inliers.size()
                            % guess_tx % guess_ty % guess_tz % guess_rx % guess_ry % guess_rz
                            % ground_tx % ground_ty % ground_tz % ground_rx % ground_ry % ground_rz 
                            % max_rerr_t % max_rerr_r << endl;

                cout << boost::format("Detected %s (%d inliers)") % name % guess.inliers.size() << endl;
            }
            objectIndex++;
        }

        if (write_store && found.empty()) {
            // Make sure that there is also a picture that denotes that no guess has been
            // made on a certain picture
            string none_name = str(boost::format("%s/%s.none.png") % opts.storeDirectory % mapped_img_name);
            Mat noCanvas = test.image.clone();
            putText(noCanvas, "No subjects detected!", Point(150, 100), FONT_HERSHEY_SIMPLEX, 1.25, 200, 2);
            imwrite(none_name, noCanvas);
        }

        if (write_store) {
            string all_guesses_name = str(boost::format("%s/%s.guesses.png") % opts.storeDirectory % mapped_img_name);
            Mat canvas;
            cvtColor(test.image, canvas, CV_GRAY2BGR);
            // TODO: coherent naming style
            drawGuesses(canvas, guesses, trainingCamera, ground_poses);
            imwrite(all_guesses_name, canvas);
        }

        if (opts.verbose >= 2) {
            foreach(const Guess & guess, guesses) {
                Mat canvas = test.image.clone();
                drawPose(canvas, guess.aligned_pose(), objects[0]->observations[0].camera());
                imshow("Guess", canvas);
                waitKey(0);
            }
        }

        // Check for true or false positive.
        foreach (string name, found) {
            if (groundTruth.onScene(name)) {
                tp += 1;
            } else {
                fp += 1;
            }
        }

        int n = objects.size() - p;
        int fn = p - tp;
        int tn = n - fp;
        if (write_stats) {
            stats << endl;
            stats << "# -- " << img_name << " -- " << endl;
            stats << "# actual objects: ";
            foreach (NamedPose np, it->second.labels) {
                stats << np.name << ", ";
            }
            stats << endl;
            stats << "# predicted objects: ";
            foreach (string x, found) {
                stats << x << ", ";
            }
            stats << endl;
            stats << "# tp = " << tp << endl;
            stats << "# fp = " << fp << endl;
            stats << "# tn = " << tn << endl;
            stats << "# fn = " << fn << endl;
            stats << "# p = " << p << endl;
            stats << "# n = " << n << endl;
        }

        n_acc += n;
        p_acc += p;
        tp_acc += tp;
        fp_acc += fp;
        fn_acc += fn;
        tn_acc += tn;

        if (write_result) {
            result << img_name << " = ";
            foreach(string obj, found) {
                result << obj << " ";
            }
            result << endl;
        }
        foreach (string obj, found) {
            cout << (boost::format("Detected %15s (%d different guesses)") % obj % guess_count[obj]) << endl;
        }
    }
    
    if (write_result) {
        result.close();
    }

    if (write_roc) {
        // gnuplot does not like NaN
        if (n_acc > 0 && p_acc > 0) {
            roc << fp_acc / (double) n_acc << " ";
            roc << tp_acc / (double) p_acc << endl;
        }
        roc << "e" << endl;
        roc << "## confusion matrix" << endl;
        roc << "##    accumulated over each test image";
        roc << "# tp = " << tp_acc << endl;
        roc << "# fp = " << fp_acc << endl;
        roc << "# tn = " << tn_acc << endl;
        roc << "# fn = " << fn_acc << endl;
        roc << "## n = tn + fp" << endl;
        roc << "# n = " << n_acc << endl;
        roc << "## p = tp + fn" << endl;
        roc << "# p = " << p_acc << endl; 
        roc << "## tp_rate = tp / p = sensitivity = hit rate = recall" << endl;
        roc << "# tp_rate = " << tp_acc / (double) p_acc << endl;
        roc << "## fp_rate = fall-out" << endl;
        roc << "# fp_rate = " << fp_acc / (double) n_acc << endl;
        roc.close();
    }

    if (write_stats) {
        // write down confusion matrix
        // plus some more calculation on top of it
        stats << endl;
        stats << endl;
        stats << "# confusion matrix" << endl;
        stats << "#    accumulated over each test image" << endl;
        stats << "tp = " << tp_acc << endl;
        stats << "fp = " << fp_acc << endl;
        stats << "tn = " << tn_acc << endl;
        stats << "fn = " << fn_acc << endl;
        stats << "# n = tn + fp" << endl;
        stats << "n = " << n_acc << endl;
        stats << "# p = tp + fn" << endl;
        stats << "p = " << p_acc << endl; 
        stats << "# tp_rate = tp / p = sensitivity = hit rate = recall" << endl;
        stats << "tp_rate = " << tp_acc / (double) p_acc << endl;
        stats << "# fp_rate = fall-out" << endl;
        stats << "fp_rate = " << fp_acc / (double) n_acc << endl;
        stats.close();
    }

    if (write_table) {
        table.close();
    }

    return 0;
}
