/*
 * Author: Julius Adorf
 */

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

#ifndef _FLAGS_H_
#define _FLAGS_H_

namespace clutseg {

    /** \brief Creates and removes an empty file as a means of inter-process
     * communication. */
    class FileFlag {

        public:

            /** \brief Dummy constructor. Invalid object returned. */
            FileFlag();

            /** \brief Initializes a flag. Does NOT set the flag. */
            FileFlag(const boost::filesystem::path & flagp);

            /** 
             * \brief Creates the file to signal the flag is set.
             *
             * If the file already exists, it will be overwritten. If a
             * directory with the same name exists, setting the flag fails
             * with an error.
             */
            void set();

            /** \brief Returns whether the flag is set. */
            bool exists();

            /** \brief Clears the flag.
             * 
             * Removes the file. If the file does not exists, this is a no-op.
             * If the file is a symlink, the symlink is removed. If a directory
             * with the same name exists, clearing the flag results in an
             * error.
             */
            void clear();

            /** \brief Returns the path to the flag file. */
            boost::filesystem::path path();

        private:
            
            boost::filesystem::path flagp_;

    };

}

#endif

