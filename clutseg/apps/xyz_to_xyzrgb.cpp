
/* 
 * Author: Julius Adorf
 */

#include "clutseg/gcc_diagnostic_disable.h"
    #include <iostream>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <boost/filesystem.hpp>
    #include <boost/algorithm/string/predicate.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace pcl;
using namespace boost::filesystem;
using namespace boost::algorithm;

// TODO: enable -Wunused-parameter for this file
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

/** \brief Reads XYZ point clouds from files in a directory and writes XYZRGB
 * point clouds back to files in the same directory.
 */
int main(int argc, char *argv[]) {
    if (argc != 2) {
        cerr << "Usage: xyz_to_xyzrgb <directory>" << endl;
        cerr << endl;
        cerr << "     where <directory> refers to a folder that contains " << endl;
        cerr << "     PCD files with XYZ point data. Original files will " << endl;
        cerr << "     not be overwritten." << endl;
        return 1;
    }
    string dir(argv[1]);
    for (directory_iterator it(dir), end; it != end; ++it) {
        string f(it->path().filename());
        if (ends_with(f, ".pcd")) {
            string fnew = f.substr(0, f.length()-4) + ".rgbxyz.pcd"; 
            cout << f << " ----> " << fnew << endl; 
            PointCloud<PointXYZ> cloud_xyz;
            PointCloud<PointXYZRGB> cloud_xyzrgb;
            io::loadPCDFile(dir + "/" + f, cloud_xyz);

            PointCloud<PointXYZRGB> cloud_xyzrgb2;
            cloud_xyzrgb2.points.resize(cloud_xyz.size());
            for (unsigned int i = 0; i < cloud_xyz.points.size(); i++) {
                cloud_xyzrgb2.points[i].x = cloud_xyz.points[i].x;
                cloud_xyzrgb2.points[i].y = cloud_xyz.points[i].y;
                cloud_xyzrgb2.points[i].z = cloud_xyz.points[i].z;
            }
            copyPointCloud(cloud_xyz, cloud_xyzrgb);
            io::savePCDFileASCII(dir + "/" + fnew, cloud_xyzrgb);
            io::savePCDFileASCII(dir + "/_" + fnew, cloud_xyzrgb2);
        }
        return 0;
    }
}

