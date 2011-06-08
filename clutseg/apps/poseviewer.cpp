/**
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"
#include "clutseg/pcl_visualization_addons.h"
#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <iostream>
    #include <boost/lexical_cast.hpp>
    #include <boost/format.hpp>
    #include <boost/algorithm/string.hpp>
    #include <opencv_candidate/PoseRT.h>
    #include <cv.h>
    #include <pcl/point_types.h>
    #include <pcl/io/pcd_io.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace pcl_visualization;
using namespace clutseg;

// TODO: enable -Wunused-parameter for this file
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

int main(int argc, char *argv[]) {
    // Read arguments
    if (argc == 0) {
        cerr << "Usage: poseviewer (<point-cloud-file>|<pose-file>)*" << endl;
        return 1;
    }

    // Create visualization
    PCLVisualizer visualizer;
    visualizer.addCoordinateSystem(0.5, 0, 0, 0);

    for (int i = 0; i < argc; i++) {
        string arg(argv[i]);
        if (boost::algorithm::ends_with(arg, ".pcd")) {
            PointCloud<PointXYZ> cloud;
            io::loadPCDFile(arg, cloud);
            cout << boost::format("%10s: %s") % "file" % arg << endl; 
            cout << boost::format("%10s: %8d") % "width" % cloud.width << endl; 
            cout << boost::format("%10s: %8d") % "height" % cloud.height << endl; 
            cout << boost::format("%10s: %8d") % "size" % cloud.size() << endl; 
            cout << boost::format("%10s: %8s") % "is_dense" % !cloud.is_dense << endl; 
            cout << endl;
            visualizer.addPointCloud(cloud, str(boost::format("cloud-%d") % i));
        } else if (boost::algorithm::ends_with(arg, ".ground.yaml")) {
            LabelSet g;
            g.read(arg);
            BOOST_FOREACH(Label np, g.labels) {
                addPose(visualizer, np.pose, str(boost::format("pose-%d-%d") % i % rand()));
            }
        } else if (boost::algorithm::ends_with(arg, ".yaml")) {
            PoseRT pose;
            FileStorage in(arg, FileStorage::READ); 
            pose.read(in[PoseRT::YAML_NODE_NAME]);
            addPose(visualizer, pose, str(boost::format("pose-%d") % i));
        }
    }
    
    // show visualization
    visualizer.spin(); 
}

