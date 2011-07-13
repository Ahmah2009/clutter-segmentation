/*
 * Author: Julius Adorf
 */

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

#ifndef _FLAGS_H_
#define _FLAGS_H_

namespace clutseg {

    class FileFlag {

        public:

            FileFlag();

            FileFlag(const boost::filesystem::path & flagp);

            void set();

            bool exists();

            void clear();

            boost::filesystem::path path();

        private:
            
            boost::filesystem::path flagp_;

    };

}

#endif

