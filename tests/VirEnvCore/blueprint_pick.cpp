// Prints what the C++ blueprint picker (CommonLib/BlueprintPick.h) decides, so
// tests/VirEnv/test_blueprint_parity.py can compare it line for line with the
// Python bridge. Three kinds of line, all generated from the same rules the
// Python test uses:
//
//   HASH <len> <hex>          BLAKE2b-64 of a length-<len> pattern string,
//                             across the 128-byte block boundaries
//   POOL <vClass> <i> <bp>    every pool, in order
//   PICK <seed> <vClass> <id> <bp>
//
// SDK-free: no Carla, no sockets, no config.

#include "BlueprintPick.h"

#include <cstdio>
#include <string>
#include <vector>

namespace bp = fixs::blueprint;

static std::string pattern(size_t n) {
    std::string s;
    for (size_t i = 0; i < n; ++i) s.push_back((char)('a' + (i % 26)));
    return s;
}

static std::string hex(const std::vector<uint8_t>& d) {
    static const char* k = "0123456789abcdef";
    std::string s;
    for (uint8_t b : d) { s.push_back(k[b >> 4]); s.push_back(k[b & 15]); }
    return s;
}

// The ids the pick is compared over: SUMO-shaped, plus the cases a hash gets
// wrong first -- one byte, and non-ASCII (UTF-8 bytes, as the wire has). Not the
// empty id: no vehicle has one, and Python answers it with its legacy random draw
// rather than the hash (HASH 0 above still checks the empty input to BLAKE2b).
static std::vector<std::string> ids() {
    std::vector<std::string> out = {"0", "ego", "7.23", "v\xc3\xa9h_1"};
    for (int i = 0; i < 300; ++i)
        out.push_back("flow_" + std::to_string(i % 7) + "." + std::to_string(i));
    return out;
}

int main() {
    for (size_t n : {0, 1, 2, 63, 64, 65, 127, 128, 129, 255, 256, 257, 300})
        std::printf("HASH %zu %s\n", n, hex(bp::blake2b(pattern(n), 8)).c_str());

    for (const std::string& vc : bp::vClasses()) {
        const std::vector<std::string>* pool = bp::poolFor(vc);
        for (size_t i = 0; i < pool->size(); ++i)
            std::printf("POOL %s %zu %s\n", vc.c_str(), i, (*pool)[i].c_str());
    }

    for (long long seed : {bp::kDefaultSeed, 5LL}) {
        for (const std::string& vc : bp::vClasses())
            for (const std::string& id : ids())
                std::printf("PICK %lld %s %s %s\n", seed, vc.c_str(), hex(
                    std::vector<uint8_t>(id.begin(), id.end())).c_str(),
                    bp::pick(vc, id, seed).c_str());
        // An unknown class falls back the same way on both sides.
        std::printf("PICK %lld %s %s %s\n", seed, "no_such_class", "6964",
                    bp::pick("no_such_class", "id", seed).c_str());
    }
    return 0;
}
