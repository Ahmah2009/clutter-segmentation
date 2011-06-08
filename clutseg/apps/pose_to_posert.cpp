/* *
 * Author: Julius Adorf
 */

#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <iostream>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <boost/filesystem.hpp>
    #include <boost/algorithm/string/predicate.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace std;
namespace bfs = boost::filesystem;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "Usage: pose_to_posert <in-file> <out-file>" << endl;
        cerr << endl;
        return 1;
    }
    bfs::path inp(argv[1]);
    bfs::path outp(argv[2]);
    convertPoseFileToDouble(inp, outp);
    return 0;
}

