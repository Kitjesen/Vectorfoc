/**
 * @file    version.h
 * @brief   Firmware version information
 * @note    Version is injected at build time via CMake from git tag
 */

#ifndef VERSION_H
#define VERSION_H

// Version components - defined by CMake, or use defaults
#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR    1
#endif

#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR    0
#endif

#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH    0
#endif

// Git info - defined by CMake
#ifndef FW_GIT_HASH
#define FW_GIT_HASH         "unknown"
#endif

#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY        0
#endif

// Formatted version string
#define FW_VERSION_STRING   STRINGIFY(FW_VERSION_MAJOR) "." \
                            STRINGIFY(FW_VERSION_MINOR) "." \
                            STRINGIFY(FW_VERSION_PATCH)

#define STRINGIFY(x)        STRINGIFY_(x)
#define STRINGIFY_(x)       #x

// Full version info (for GET_VERSION command)
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t dirty;      // 1 if uncommitted changes
    char git_hash[8];   // Short git hash
} FirmwareVersion_t;

/**
 * @brief Get firmware version info
 * @return Pointer to static version struct
 */
static inline const FirmwareVersion_t* FW_GetVersion(void) {
    static const FirmwareVersion_t version = {
        .major = FW_VERSION_MAJOR,
        .minor = FW_VERSION_MINOR,
        .patch = FW_VERSION_PATCH,
        .dirty = FW_GIT_DIRTY,
        .git_hash = FW_GIT_HASH
    };
    return &version;
}

#endif // VERSION_H
