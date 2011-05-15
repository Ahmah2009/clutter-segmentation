/**
 * Author: Julius Adorf
 */

#include <boost/filesystem.hpp>

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

