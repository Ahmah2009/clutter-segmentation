/**
 * Author: Julius Adorf
 */

#include "clutseg/flags.h"

#include <fstream>
#include <iostream>

using namespace std;

namespace bfs = boost::filesystem;

namespace clutseg {

    FileFlag::FileFlag() {}

    FileFlag::FileFlag(const bfs::path & flagp) : flagp_(flagp) {}

    void FileFlag::set() {
        if (bfs::is_directory(flagp_)) {
            throw runtime_error("Cannot set flag. A directory with the same name already exists!");
        }
        ofstream f;
        f.open(flagp_.string().c_str());
        f.close();
    }

    bool FileFlag::exists() {
        return bfs::exists(flagp_) && !bfs::is_directory(flagp_);
    }

    void FileFlag::clear() {
        if (bfs::is_directory(flagp_)) {
            throw runtime_error("Cannot clear flag. A directory with the same name already exists!");
        }
        bfs::remove(flagp_);
    }

    bfs::path FileFlag::path() {
        return flagp_;    
    }

}
