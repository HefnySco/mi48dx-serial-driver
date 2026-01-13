#ifndef VERSION_H
#define VERSION_H

#include <string>

// This will be filled by CMake
#ifndef __APP__VERSION__
#define __APP__VERSION__ "1.0.0"
#endif

namespace MI48 {
    // Library version information
    static constexpr const char* VERSION_STRING = __APP__VERSION__;
    
    // Helper function to get version string
    inline std::string get_version() {
        return VERSION_STRING;
    }
}

#endif // VERSION_H