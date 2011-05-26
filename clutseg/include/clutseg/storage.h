/**
 * Author: Julius Adorf
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include "clutseg/ground.h" 
#include "clutseg/report.h" 
#include "clutseg/result.h" 

#include <boost/filesystem.hpp>
#include <cv.h>
#include <opencv_candidate/Camera.h>

namespace clutseg {

    class ResultStorage {

        public:

            ResultStorage() : result_dir_("") {}
            ResultStorage(const boost::filesystem::path & result_dir) : result_dir_(result_dir) {}

            /** Stores results for one test scene. */
            void store(const TestReport & report); // TODO: rename store -> record

        private:

            boost::filesystem::path result_dir_;

    };

}

#endif
