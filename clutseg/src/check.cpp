/**
 * Author: Julius Adorf
 */

#include "clutseg/check.h"

#include <boost/format.hpp>
#include <exception>

using namespace cv;
using namespace std;
namespace bfs = boost::filesystem;

namespace clutseg {

   void assert_path_exists(const bfs::path & path) {
        if (!bfs::exists(path)) {
            throw runtime_error(str(boost::format("Path '%s' does not exist") % path));
        }
   } 

   void assert_valid_image(const cv::Mat & img) {
        if (img.empty()) {
            throw runtime_error("Invalid image. Empty.");
        }
   } 

}
