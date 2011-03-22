
/* 
 * Author: Julius Adorf
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

using namespace std;
using namespace pcl;
using namespace boost::filesystem;
using namespace boost::algorithm;

/** \brief Reads XYZ point clouds from files in a directory and writes XYZRGB
 * point clouds back to files in the same directory.
 */
int main(int argc, char *argv[])
{
    string dir(argv[1]);
    for (directory_iterator it(dir), end; it != end; ++it) {
        string f(it->path().filename());
        if (ends_with(f, ".pcd")) {
            string fnew = f.substr(0, f.length()-4) + ".rgbxyz.pcd"; 
            cout << f << " ----> " << fnew << endl; 
            PointCloud<PointXYZ> cloud_xyz;
            PointCloud<PointXYZRGB> cloud_xyzrgb;
            io::loadPCDFile(dir + "/" + f, cloud_xyz);
            cloud_xyzrgb.points.resize(cloud_xyz.size());
            for (unsigned int i = 0; i < cloud_xyz.points.size(); i++) {
                cloud_xyzrgb.points[i].x = cloud_xyz.points[i].x;
                cloud_xyzrgb.points[i].y = cloud_xyz.points[i].y;
                cloud_xyzrgb.points[i].z = cloud_xyz.points[i].z;
            }
            io::savePCDFileASCII(dir + "/" + fnew, cloud_xyzrgb);
        }
    }
/* 
    PointCloud<PointXYZ> cloud_xyz;
    PointCloud<PointXYZRGB> cloud_xyzrgb;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud_xyz);
    cloud_xyzrgb.points.resize(cloud_xyz.size());
    for (unsigned int i = 0; i < cloud_xyz.points.size(); i++) {
        cloud_xyzrgb.points[i].x = cloud_xyz.points[i].x;
        cloud_xyzrgb.points[i].y = cloud_xyz.points[i].y;
        cloud_xyzrgb.points[i].z = cloud_xyz.points[i].z;
    }
    io::savePCDFileASCII("build/sample.delimited.xyzrgb.pcd", cloud_xyzrgb); */
}

