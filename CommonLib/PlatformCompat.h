#pragma once
// ---------------------------------------------------------------------------
// PlatformCompat -- the small Win32 surface that CommonLib and TrafficLayer
// call directly, with a POSIX implementation beside it (issue #65).
//
// Two deliberate constraints:
//
//   * HEADER-ONLY. Five .vcxproj files compile ConfigHelper.cpp -- two of them
//     live in the branch-protected ProprietaryFiles submodule. A new .cpp TU
//     would mean editing all five (and a submodule PR); an inline header means
//     none of them change at all.
//
//   * C++14, no <filesystem>. CommonLib is consumed by the CarMaker/dSPACE RT
//     targets, which is the reason ConfigHelper hand-rolled these helpers on
//     shlwapi in the first place (#65 Q4). That constraint is kept, not relaxed.
//
// The Win32 bodies below are the ones that were previously inline in
// ConfigHelper.cpp / mainTrafficLayer.cpp, moved verbatim -- Windows behaviour
// is unchanged. Only the POSIX side is new.
// ---------------------------------------------------------------------------

#include <string>
#include <vector>
#include <ctime>

#ifdef _WIN32
    #include <windows.h>
    #include <shlwapi.h>
    #pragma comment(lib, "shlwapi.lib")
#else
    #include <unistd.h>
    #include <sys/stat.h>
    #include <sys/types.h>
    #include <cerrno>
    #include <cstring>
#endif

namespace FIXS {
namespace Platform {

// --- paths -----------------------------------------------------------------

#ifndef _WIN32
// Purely LEXICAL normalisation -- resolves "." and ".." without touching the
// filesystem. This matches PathCanonicalizeA, which is also lexical; realpath()
// would NOT, because it fails on paths that do not exist yet (log files, output
// dirs), which is exactly how this is used.
inline std::string lexicalNormalize(const std::string& in) {
    const bool absolute = !in.empty() && (in[0] == '/' || in[0] == '\\');
    std::vector<std::string> parts;
    std::string cur;
    for (size_t i = 0; i <= in.size(); ++i) {
        const char c = (i < in.size()) ? in[i] : '/';
        if (c == '/' || c == '\\') {
            if (cur.empty() || cur == ".") {
                // skip
            } else if (cur == "..") {
                if (!parts.empty() && parts.back() != "..") parts.pop_back();
                else if (!absolute) parts.push_back("..");
            } else {
                parts.push_back(cur);
            }
            cur.clear();
        } else {
            cur += c;
        }
    }
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i) out += '/';
        out += parts[i];
    }
    if (absolute) out = "/" + out;
    if (out.empty()) out = absolute ? "/" : ".";
    return out;
}
#endif

inline std::string currentDirectory() {
#ifdef _WIN32
    char buffer[MAX_PATH];
    GetCurrentDirectoryA(MAX_PATH, buffer);
    return std::string(buffer);
#else
    char buffer[4096];
    if (getcwd(buffer, sizeof(buffer)) == nullptr) return std::string(".");
    return std::string(buffer);
#endif
}

inline std::string combinePaths(const std::string& base, const std::string& relative) {
#ifdef _WIN32
    char combined[MAX_PATH];
    PathCombineA(combined, base.c_str(), relative.c_str());
    char normalized[MAX_PATH];
    PathCanonicalizeA(normalized, combined);
    return std::string(normalized);
#else
    if (!relative.empty() && (relative[0] == '/' || relative[0] == '\\'))
        return lexicalNormalize(relative);
    if (base.empty()) return lexicalNormalize(relative);
    return lexicalNormalize(base + "/" + relative);
#endif
}

// --- filesystem probes ------------------------------------------------------

// true if the directory now exists -- whether this call created it or it was
// already there (the ERROR_ALREADY_EXISTS case the callers all special-cased).
inline bool createDirectory(const std::string& path) {
#ifdef _WIN32
    return CreateDirectoryA(path.c_str(), NULL) != 0 ||
           GetLastError() == ERROR_ALREADY_EXISTS;
#else
    return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

// true if 'path' exists AND is a regular file (not a directory).
inline bool fileExists(const std::string& path) {
#ifdef _WIN32
    const DWORD attrib = GetFileAttributesA(path.c_str());
    return attrib != INVALID_FILE_ATTRIBUTES &&
           !(attrib & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat st;
    return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
#endif
}

inline bool directoryExists(const std::string& path) {
#ifdef _WIN32
    const DWORD attrib = GetFileAttributesA(path.c_str());
    return attrib != INVALID_FILE_ATTRIBUTES &&
           (attrib & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat st;
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

// --- misc -------------------------------------------------------------------

inline void sleepMs(unsigned int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec ts;
    ts.tv_sec  = static_cast<time_t>(ms / 1000u);
    ts.tv_nsec = static_cast<long>((ms % 1000u) * 1000000ul);
    nanosleep(&ts, nullptr);
#endif
}

// ctime(3) with the platform's re-entrant spelling. Keeps ctime's trailing
// newline, which the existing log lines rely on.
inline std::string formatCTime(std::time_t t) {
    char buf[100];
#ifdef _WIN32
    ctime_s(buf, sizeof buf, &t);
#else
    ctime_r(&t, buf);
#endif
    return std::string(buf);
}

// --- socket errors ----------------------------------------------------------

inline int socketErrorCode() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

inline std::string socketErrorText(int code) {
#ifdef _WIN32
    return std::to_string(code);
#else
    return std::string(std::strerror(code));
#endif
}

#ifdef _WIN32
    const int kSocketErrInterrupted = WSAEINTR;
    const int kSocketErrFault       = WSAEFAULT;
#else
    const int kSocketErrInterrupted = EINTR;
    const int kSocketErrFault       = EFAULT;
#endif

} // namespace Platform
} // namespace FIXS
