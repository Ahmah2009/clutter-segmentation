/**
 * Author: Julius Adorf
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include "clutseg/ground.h" 
#include "clutseg/report.h" 
#include "clutseg/result.h" 

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <cv.h>
    #include <opencv_candidate/Camera.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    class ResultStorage {

        public:

            ResultStorage() : result_dir_("") {}
            ResultStorage(const boost::filesystem::path & result_dir) : result_dir_(result_dir) {}

            /** Stores results for one test scene. */
            void record(const TestReport & report);

        private:

            boost::filesystem::path result_dir_;

    };

}

#endif
