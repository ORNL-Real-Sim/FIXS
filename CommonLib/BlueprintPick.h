#pragma once
// Which CARLA blueprint a mirrored traffic vehicle gets -- the C++ twin of
// Carla/VirEnv/BridgeHelper.py's map_Sumo_vClass_to_Carla_blueprintId.
//
// The blueprint is not cosmetic. It fixes bounding_box.extent.x, and the bridge
// anchors a vehicle by stepping back that half-length from the nose the wire
// carries, so two models put one car in two places for its whole life. The old
// draw here took the next value from a shared random generator, so a vehicle's
// model depended on how many vehicles had drawn before it; a refused spawn
// re-dealt every later one. Measured on MLK (Python side, FIXS#358): nine
// refusals, 46 vehicles re-modelled, up to 1.58 m each, the ego 150 m off its
// baseline. Keyed on the vehicle's id, nothing but WHICH vehicle it is decides.
//
// It must agree with Python byte for byte, because the two bridges drive the
// same scenarios and are compared against each other. Hence the hash is
// Python's -- BLAKE2b, 8-byte digest, of "<seed>|<vehId>" in UTF-8, read
// big-endian, modulo the pool size -- rather than something shorter to write:
// a different hash would re-deal every vehicle in every seeded baseline the
// Python bridge has already produced, which is the damage #358 fixed.
// tests/VirEnv/test_blueprint_parity.py holds the two to that, including
// BLAKE2b itself against hashlib across the block boundaries.
//
// SDK-free on purpose (no Carla headers), so the parity test can compile it
// without libcarla -- which today does not compile at all (#380).

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace fixs {
namespace blueprint {

// ---------------------------------------------------------------- BLAKE2b
// RFC 7693, unkeyed, variable digest length. Only what the picker needs.
namespace detail {

inline uint64_t rotr64(uint64_t x, int n) { return (x >> n) | (x << (64 - n)); }

inline uint64_t load64le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}

inline void compress(uint64_t h[8], const uint8_t block[128], uint64_t t, bool last) {
    static const uint64_t IV[8] = {
        0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
        0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
        0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL};
    static const uint8_t SIGMA[12][16] = {
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
        {14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3},
        {11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4},
        {7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8},
        {9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13},
        {2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9},
        {12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11},
        {13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10},
        {6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5},
        {10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0},
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
        {14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3}};

    uint64_t m[16], v[16];
    for (int i = 0; i < 16; ++i) m[i] = load64le(block + 8 * i);
    for (int i = 0; i < 8; ++i) { v[i] = h[i]; v[i + 8] = IV[i]; }
    v[12] ^= t;                       // byte counter, low word (inputs here are < 2^64)
    if (last) v[14] = ~v[14];

    auto G = [&](int a, int b, int c, int d, uint64_t x, uint64_t y) {
        v[a] = v[a] + v[b] + x; v[d] = rotr64(v[d] ^ v[a], 32);
        v[c] = v[c] + v[d];     v[b] = rotr64(v[b] ^ v[c], 24);
        v[a] = v[a] + v[b] + y; v[d] = rotr64(v[d] ^ v[a], 16);
        v[c] = v[c] + v[d];     v[b] = rotr64(v[b] ^ v[c], 63);
    };
    for (int r = 0; r < 12; ++r) {
        const uint8_t* s = SIGMA[r];
        G(0, 4, 8, 12, m[s[0]], m[s[1]]);   G(1, 5, 9, 13, m[s[2]], m[s[3]]);
        G(2, 6, 10, 14, m[s[4]], m[s[5]]);  G(3, 7, 11, 15, m[s[6]], m[s[7]]);
        G(0, 5, 10, 15, m[s[8]], m[s[9]]);  G(1, 6, 11, 12, m[s[10]], m[s[11]]);
        G(2, 7, 8, 13, m[s[12]], m[s[13]]); G(3, 4, 9, 14, m[s[14]], m[s[15]]);
    }
    for (int i = 0; i < 8; ++i) h[i] ^= v[i] ^ v[i + 8];
}

}  // namespace detail

// BLAKE2b digest of `data`, `outlen` bytes (1..64), as hashlib.blake2b(data,
// digest_size=outlen).digest() returns it.
inline std::vector<uint8_t> blake2b(const std::string& data, size_t outlen) {
    uint64_t h[8] = {
        0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
        0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
        0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL};
    h[0] ^= 0x01010000ULL ^ (uint64_t)outlen;          // depth 1, fanout 1, no key

    const uint8_t* p = reinterpret_cast<const uint8_t*>(data.data());
    size_t len = data.size(), off = 0;
    uint64_t t = 0;
    while (len - off > 128) {                           // every block but the last
        t += 128;
        detail::compress(h, p + off, t, false);
        off += 128;
    }
    uint8_t block[128];
    std::memset(block, 0, sizeof block);
    std::memcpy(block, p + off, len - off);
    t += (uint64_t)(len - off);
    detail::compress(h, block, t, true);                // the last, zero-padded

    std::vector<uint8_t> out(outlen);
    for (size_t i = 0; i < outlen; ++i) out[i] = (uint8_t)(h[i / 8] >> (8 * (i % 8)));
    return out;
}

// ---------------------------------------------------------------- pools
// In Python's order, which is what the index is taken against. Membership AND
// order are compared by the parity test; either drifting re-deals vehicles.
inline const std::vector<std::string>* poolFor(const std::string& vClass) {
    static const std::vector<std::string> cars = {
        "vehicle.audi.a2", "vehicle.audi.etron", "vehicle.audi.tt",
        "vehicle.bmw.grandtourer", "vehicle.chevrolet.impala", "vehicle.citroen.c3",
        "vehicle.dodge.charger_2020", "vehicle.ford.crown", "vehicle.ford.mustang",
        "vehicle.jeep.wrangler_rubicon", "vehicle.lincoln.mkz_2017",
        "vehicle.lincoln.mkz_2020", "vehicle.mercedes.coupe",
        "vehicle.mercedes.coupe_2020", "vehicle.micro.microlino",
        "vehicle.mini.cooper_s", "vehicle.mini.cooper_s_2021", "vehicle.nissan.micra",
        "vehicle.nissan.patrol", "vehicle.nissan.patrol_2021", "vehicle.seat.leon",
        "vehicle.tesla.model3", "vehicle.toyota.prius"};
    static const std::vector<std::string> trucks = {
        "vehicle.carlamotors.carlacola", "vehicle.carlamotors.european_hgv",
        "vehicle.tesla.cybertruck"};
    static const std::vector<std::string> vans = {
        "vehicle.mercedes.sprinter", "vehicle.volkswagen.t2",
        "vehicle.volkswagen.t2_2021"};
    static const std::vector<std::string> buses = {"vehicle.mitsubishi.fusorosa"};
    static const std::vector<std::string> motorcycles = {
        "vehicle.harley-davidson.low_rider", "vehicle.kawasaki.ninja",
        "vehicle.vespa.zx125", "vehicle.yamaha.yzf"};
    static const std::vector<std::string> bicycles = {
        "vehicle.bh.crossbike", "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets"};
    static const std::vector<std::string> pedestrians = {
        "walker.pedestrian.0001", "walker.pedestrian.0002", "walker.pedestrian.0003",
        "walker.pedestrian.0004", "walker.pedestrian.0005"};
    // CARLA does not separate emergency vehicles by type: van / truck / car.
    static const std::vector<std::string> emergency = {
        "vehicle.ford.ambulance", "vehicle.carlamotors.firetruck",
        "vehicle.dodge.charger_police", "vehicle.dodge.charger_police_2020"};

    if (vClass == "passenger")  return &cars;
    if (vClass == "truck")      return &trucks;
    if (vClass == "van")        return &vans;
    if (vClass == "bus")        return &buses;
    if (vClass == "motorcycle") return &motorcycles;
    if (vClass == "bicycle")    return &bicycles;
    if (vClass == "pedestrian") return &pedestrians;
    if (vClass == "emergency")  return &emergency;
    return nullptr;
}

// Every vClass with a pool, in the order the parity test enumerates them.
inline const std::vector<std::string>& vClasses() {
    static const std::vector<std::string> all = {
        "passenger", "truck", "van", "bus", "motorcycle", "bicycle",
        "pedestrian", "emergency"};
    return all;
}

// Python's _kBlueprintSeed. CarlaSetup.BlueprintSeed is meant to override it on
// both sides; neither side reads that key today.
constexpr long long kDefaultSeed = 20260913;

// ---------------------------------------------------------------- the pick
// A stable index into a pool of n from (seed, vehId) -- Python's
// BridgeHelper._blueprintIndexOf.
inline size_t indexOf(const std::string& vehId, size_t n, long long seed = kDefaultSeed) {
    std::vector<uint8_t> d = blake2b(std::to_string(seed) + "|" + vehId, 8);
    uint64_t v = 0;
    for (uint8_t b : d) v = (v << 8) | b;              // big-endian, as int.from_bytes
    return (size_t)(v % (uint64_t)n);
}

// The blueprint id for one vehicle. Unknown vClass falls back to a passenger car
// and reports it through `unknown`, so the caller decides how loudly.
inline std::string pick(const std::string& vClass, const std::string& vehId,
                        long long seed = kDefaultSeed, bool* unknown = nullptr) {
    const std::vector<std::string>* pool = poolFor(vClass);
    if (unknown) *unknown = (pool == nullptr);
    if (pool == nullptr) return "vehicle.tesla.model3";
    return (*pool)[indexOf(vehId, pool->size(), seed)];
}

}  // namespace blueprint
}  // namespace fixs
