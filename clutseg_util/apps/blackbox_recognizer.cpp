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
 * This implementation fixes some of the issues.
 */

#include "tod/detecting/Loader.h"
#include "tod/detecting/Recognizer.h"
#include "testdesc.h"
#include "mute.h"

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

#define foreach BOOST_FOREACH

using namespace cv;
using namespace tod;
using namespace std;
namespace po = boost::program_options;

namespace {
    struct Options
    {
        std::string imageDirectory;
        std::string baseDirectory;
        std::string config;
        std::string testdescFilename;
        std::string resultFilename;
        std::string statsFilename;
        std::string logFilename;
        std::string rocFilename;
        TODParameters params;
        int verbose;
        int mode;
    };
}

int options(int ac, char **av, Options & opts)
{

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "Produce help message.");
    desc.add_options()("image,I", po::value < string > (&opts.imageDirectory),
                       "Test image base directory.");
    desc.add_options()("testdesc",
                       po::value < string > (&opts.testdescFilename),
                       "Test description file");
    desc.add_options()("base,B",
                       po::value < string >
                       (&opts.baseDirectory)->default_value("./"),
                       "The directory that the training base is in.");
    desc.add_options()("tod_config,f", po::value < string > (&opts.config),
                       "The name of the configuration file");
    desc.add_options()("log,l", po::value < string > (&opts.logFilename),
                       "The name "
                       "of the log file where results are written to in YAML format. Cannot be written "
                       "to standard output because standard output seems to serve debugging "
                       "purposes...");
    desc.add_options()("result,r",
                       po::value < string > (&opts.resultFilename),
                       "Result file using INI-style/Python config file syntax. This one is optional.");
    desc.add_options()("stats,s",
                       po::value < string > (&opts.statsFilename),
                       "Statistics are stored in this file. INI-style/Python config file syntax.");
    desc.add_options()("roc",
                       po::value < string > (&opts.rocFilename),
                       "Generate points on a ROC graph and save it to a file.");
    desc.add_options()("verbose,V",
                       po::value < int >(&opts.verbose)->default_value(1),
                       "Verbosity level");
    desc.add_options()("mode,m",
                       po::value < int >(&opts.mode)->default_value(1),
                       "Mode");

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    FileStorage fs;
    if (opts.config.empty()
        || !(fs = FileStorage(opts.config, FileStorage::READ)).isOpened()) {
        cout << "Must supply configuration." << "\n";
        cout << desc << endl;
        return 1;
    } else
        opts.params.read(fs[TODParameters::YAML_NODE_NAME]);

    if (!vm.count("image")) {
        cout << "Must supply an image directory." << "\n";
        cout << desc << endl;
        return 1;
    }

    if (!vm.count("testdesc")) {
        cout << "Must supply a test description file." << "\n";
        cout << desc << endl;
        return 1;
    }

    if (!vm.count("base")) {
        cout << "Must supply training base directory." << "\n";
        cout << desc << endl;
        return 1;
    }

    return 0;

}

int main(int argc, char *argv[])
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    tod::Loader loader(opts.baseDirectory);
    vector < cv::Ptr < TexturedObject > >objects;
    loader.readTexturedObjects(objects);

    if (!objects.size()) {
        cout << "Empty base\n" << endl;
    }

    TrainingBase base(objects);
    Ptr < FeatureExtractor > extractor =
        FeatureExtractor::create(opts.params.feParams);

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
    }

    TestDesc testdesc = loadTestDesc(opts.testdescFilename);

    FileStorage fs;
    fs.open(opts.logFilename, FileStorage::WRITE);
    fs << "trainFolder" << opts.baseDirectory;
    fs << "test1" << "{";
    fs << "testFolder" << opts.imageDirectory;
    fs << "objects" << "{";

    bool writeR = (opts.resultFilename != "");
    fstream r;
    if (writeR) {
        r.open(opts.resultFilename.c_str(), ios_base::out);
        r << "[predictions]" << endl;
    }
   
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
    bool writeS = (opts.statsFilename != "");
    fstream s;
    if (writeS) {
        s.open(opts.statsFilename.c_str(), ios_base::out);
        s << "[statistics]" << endl;
    }

    bool writeRoc = (opts.rocFilename != "");
    fstream roc;
    if (writeRoc) {
        roc.open(opts.rocFilename.c_str(), ios_base::out);
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

    int objectIndex = 1;

    for (TestDesc::iterator it = testdesc.begin(), end = testdesc.end();
         it != end; it++) {
        // true positives
        int tp = 0;
        // false positives
        int fp = 0;
        // total positives
        int p = it->second.size();

        std::string image_name = it->first;
        string path = opts.imageDirectory + "/" + image_name;
        cout << "< Reading the image... " << path;

        Features2d test;
        test.image = imread(path, 0);
        cout << ">" << endl;
        if (test.image.empty()) {
            cout << "Cannot read test image" << endl;
            break;
        }
        extractor->detectAndExtract(test);

        Mat drawImage;
        if (test.image.channels() > 1)
            test.image.copyTo(drawImage);
        else
            cvtColor(test.image, drawImage, CV_GRAY2BGR);

        vector < tod::Guess > foundObjects;
        recognizer->match(test, foundObjects);

        if (writeR) {
            r << image_name << " = ";
        }

        set < string > found;
        foreach(const Guess & guess, foundObjects)
        {
            stringstream nodeIndex;
            nodeIndex << objectIndex++;
            string name = guess.getObject()->name;
            string nodeName = "object" + nodeIndex.str();
            fs << nodeName << "{";
            fs << "id" << guess.getObject()->id;
            fs << "name" << name;
            //fs << "r" << guess.aligned_pose().r();
            //fs << "t" << guess.aligned_pose().t();
#if 0
            int index = atoi((image_name.substr(0, position)).c_str());
            fs << "imageIndex" << index;
#endif
            fs << "imageName" << image_name;
            // damn nasty bug
            //fs << "idx" <<  guess.image_indices_;
            fs << "}";
            found.insert(name);
        }
        foreach(string name, found) {
            // Check for true or false positive.
            if (it->second.find(name) == it->second.end()) {
                fp += 1;
            } else {
                tp += 1;
            }
        }

        int n = objects.size() - p;
        int fn = p - tp;
        int tn = n - fp;
        if (writeS) {
            s << endl;
            s << "# -- " << image_name << " -- " << endl;
            s << "# actual objects: ";
            foreach (string x, it->second) {
                s << x << ", ";
            }
            s << endl;
            s << "# predicted objects: ";
            foreach (string x, found) {
                s << x << ", ";
            }
            s << endl;
            s << "# tp = " << tp << endl;
            s << "# fp = " << fp << endl;
            s << "# tn = " << tn << endl;
            s << "# fn = " << fn << endl;
            s << "# p = " << p << endl;
            s << "# n = " << n << endl;
        }

        n_acc += n;
        p_acc += p;
        tp_acc += tp;
        fp_acc += fp;
        fn_acc += fn;
        tn_acc += tn;

        if (writeR) {
            foreach(string obj, found) {
                r << obj << " ";
            }
        }
        if (writeR) {
            r << endl;
        }
    }
    fs << "objectsCount" << objectIndex - 1;
    fs << "}" << "}";
    fs.release();
    if (writeR) {
        r.close();
    }
    if (writeS) {
        // write down confusion matrix
        // plus some more calculation on top of it
        s << endl;
        s << endl;
        s << "# confusion matrix" << endl;
        s << "#    accumulated over each test image" << endl;
        s << "tp = " << tp_acc << endl;
        s << "fp = " << fp_acc << endl;
        s << "tn = " << tn_acc << endl;
        s << "fn = " << fn_acc << endl;
        s << "# n = tn + fp" << endl;
        s << "n = " << n_acc << endl;
        s << "# p = tp + fn" << endl;
        s << "p = " << p_acc << endl; 
        s << "# tp_rate = tp / p = sensitivity = hit rate = recall" << endl;
        s << "tp_rate = " << tp_acc / (double) p_acc << endl;
        s << "# fp_rate = fall-out" << endl;
        s << "fp_rate = " << fp_acc / (double) n_acc << endl;
        s.close();
    }
    if (writeRoc) {
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

    return 0;
}
