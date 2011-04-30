/* pose_estimator
 */

#include <tod/training/stats.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <fiducial/fiducial.h>

#include <tod/training/file_io.h>
#include <tod/training/Opts.h>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace fiducial;
using namespace std;
using namespace cv;

namespace po = boost::program_options;

struct pe_options
{
    CommonOptions common;
    CameraOptions camera;
    ImagesOptions images;

    std::string fiducial_yaml;
};

int options(int ac, char **av, pe_options & opts)
{
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add(opts.common.desc);
    desc.add(opts.camera.desc);
    desc.add(opts.images.desc);

    desc.add_options()("fiducial,F",
                       po::value < string >
                       (&opts.fiducial_yaml)->default_value("fiducial.yml"),
                       "The yaml file describing the fiducial marker, use gen_fiducial to see a sample yml file based on art/board.02.svg.");

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    try {
        opts.common.check(vm);
    }
    catch(std::runtime_error e) {
        cout << desc << endl;
        cerr << e.what() << endl;
        return 1;
    }
    return 0;
}

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

#define filename(arg) (directory+"/"+arg)

#define mout(x) {boost::mutex::scoped_lock sl(cout_lock);cout << x;}

#define sync(lock, code) { boost::mutex::scoped_lock sl(lock); code; }

struct pose_worker
{
    pose_worker(const std::string & directory, const list < string > &images,
                ImageBasedPE * posest, const Camera & camera, bool verbose,
                posest_stats & stats):directory(directory), images(images),
        posest(posest), camera(camera), verbose(verbose), stats(stats)
    {

    }
    void operator() ()
    {
        Mat poseImage;
        foreach(const string & x, images)
        {
            // TODO sync(stats_lock, stats.img_cnt++);

            sync(stats_lock, stats.img_cnt++);

            std::cout << "[POSEST] " << x << std::endl;

            cv::Mat orig = imread(filename(x), CV_LOAD_IMAGE_GRAYSCALE);
            // julius: take preprocessed file for pose estimation to improve success
            // rate
            cv::Mat fallback =
                imread(filename(x + ".pre.pose.png"),
                       CV_LOAD_IMAGE_GRAYSCALE);
            if (orig.empty()) {
                std::cerr << "bad file" << filename(x) << std::endl;
                continue;
            }
            //mout(".");
            posest->current_image_name = filename(x);
            PoseRT pose;

            // Try to recognize pose on original image first.
            pose = posest->estimatePose(orig);
            if (pose.estimated) {
                sync(stats_lock, stats.orig_success_cnt++);
            } else {
                sync(stats_lock, stats.orig_failure_cnt++);
            }

            if (!pose.estimated) {
                // Pose estimation on original image has failed.
                // Fallback.
                mout("Falling back to prepared pose image\n");
                pose = posest->estimatePose(fallback);
                if (pose.estimated) {
                    sync(stats_lock,
                                 stats.fallback_success_cnt++);
                } else {
                    sync(stats_lock,
                                 stats.fallback_failure_cnt++);
                }
            }

            if (pose.estimated) {
                sync(stats_lock, stats.success_cnt++);
                std::string pose_file = x + ".pose.yaml";

                FileStorage fs(filename(pose_file), FileStorage::WRITE);
                fs << PoseRT::YAML_NODE_NAME;
                pose.write(fs);
                if (verbose) {
                    cvtColor(orig, poseImage, CV_GRAY2RGB);
                    PoseDrawer(poseImage, camera.K, pose);
                    boost::mutex::scoped_lock sl(outfile_lock);
                    namedWindow("pose", CV_WINDOW_KEEPRATIO);
                    imshow("pose", poseImage);
                    waitKey(30);
                }
            } else {
                sync(stats_lock, stats.failure_cnt++);
                std::
                    cerr << "[POSEST] FAILED to estimate pose for " << x <<
                    endl;
            }
        }
    }

    static boost::mutex cout_lock;
    static boost::mutex stats_lock;
    static boost::mutex outfile_lock;
    string directory;

    list < string > images;
    ImageBasedPE *posest;
    Camera camera;
    bool verbose;
    posest_stats & stats;
};

boost::mutex pose_worker::cout_lock;
boost::mutex pose_worker::stats_lock;
boost::mutex pose_worker::outfile_lock;

int main(int argc, char *argv[])
{
    pe_options opts;
    if (options(argc, argv, opts))
        return 1;

    //read in the image list provided on the command line

    list < string > images = opts.images.loadImageNames(opts.common);
    if (images.empty()) {
        return 1;
    }

    Camera camera = opts.camera.loadCamera(opts.common);

    Ptr < ImageBasedPE > estimator;

    cv::FileStorage fs(opts.fiducial_yaml, FileStorage::READ);
    Fiducial f;
    f.read(fs["fiducial"]);
    estimator = new FiducialPoseEstimator(f, camera, false);

    //spawn some threads so that we get this done in a reasonable amount of time

    //first split the image list into the number of threads that we want
    std::vector < std::list < std::string > >vlist =
        splitList(images, opts.common.n_threads);

    posest_stats stats;
    clock_t before = clock();

    //create a thread for each sublist
    boost::thread_group threads;
    foreach(const std::list < std::string > &x, vlist) {
        threads.
            create_thread(pose_worker
                          (opts.common.directory, x, &*estimator, camera,
                           opts.common.verbose, stats));
    }

    //join all the threads that we spawned
    threads.join_all();
    clock_t after = clock();
    stats.time = float (after - before) / CLOCKS_PER_SEC;

    cout << stats;
    ofstream stats_out((opts.common.directory + "/posest_stats.yaml").c_str());
    stats_out << stats;
    stats_out.close();

    return 0;
}
