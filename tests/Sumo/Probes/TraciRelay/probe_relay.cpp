// #356 probe: can TrafficLayer execute an arbitrary TraCI command, given only the
// four values the Python seam produces (cmdID, varID, objID, packed payload bytes),
// on the SAME libtraci connection it already owns?
//
// This is a throwaway probe, not shippable code. In particular it declares
// libtraci::Connection locally (see FINDINGS.md, "packaging"): the real
// libtraci/Connection.h cannot be included because it drags in utils/common/*,
// which is not part of the shipped native-deps set. The local declaration binds to
// MSVC name mangling - fine for an experiment, not for production.
//
// usage: probe_relay.exe <port> <steps> <requests.txt> <responses.txt>
//   requests.txt  lines: name|cmdID|varID|objID|payloadHex
//   responses.txt lines: name|OK|<remaining bytes of doCommand's Storage, hex>
//                        name|EXC|<what()>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>

#include <libsumo/libtraci.h>
#include <foreign/tcpip/storage.h>

// --- minimal binding to the generic executor exported by libtracicpp.dll --------
namespace libtraci {
class Connection {
public:
    static Connection& getActive();
    std::mutex& getMutex() const;
    tcpip::Storage& doCommand(int command, int var, const std::string& id,
                              tcpip::Storage* add, int expectedType);
};
}

static std::vector<unsigned char> fromHex(const std::string& h) {
    std::vector<unsigned char> out;
    for (size_t i = 0; i + 1 < h.size(); i += 2) {
        out.push_back((unsigned char)strtol(h.substr(i, 2).c_str(), nullptr, 16));
    }
    return out;
}

static std::string toHex(const std::vector<unsigned char>& b) {
    static const char* d = "0123456789abcdef";
    std::string s;
    for (unsigned char c : b) { s += d[c >> 4]; s += d[c & 0xf]; }
    return s;
}

static std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream iss(s);
    while (std::getline(iss, cur, sep)) out.push_back(cur);
    return out;
}

int main(int argc, char** argv) {
    if (argc < 5) { printf("usage: probe_relay <port> <steps> <requests> <responses>\n"); return 2; }
    const int port = atoi(argv[1]);
    const int steps = atoi(argv[2]);

    printf("[probe] connecting to SUMO on port %d\n", port);
    auto ver = libtraci::Simulation::init(port);
    printf("[probe] connected: TraCI API %d, %s\n", ver.first, ver.second.c_str());

    // TrafficLayer drives its feed from subscriptions: relaying must not disturb them,
    // so subscribe before any relayed command and read the results back afterwards
    for (int i = 0; i + 1 < steps; ++i) libtraci::Simulation::step();
    libtraci::Vehicle::subscribe("ego", {libsumo::VAR_SPEED, libsumo::VAR_LANE_INDEX});
    libtraci::Simulation::step();
    printf("[probe] stepped %d, sim time %.2f\n", steps, libtraci::Simulation::getTime());

    // ground truth through libtraci's own typed API, before any relayed command
    printf("[probe] baseline ego speed=%.6f lane=%d\n",
           libtraci::Vehicle::getSpeed("ego"), libtraci::Vehicle::getLaneIndex("ego"));

    std::ifstream in(argv[3]);
    std::ofstream out(argv[4]);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto f = split(line, '|');
        if (f.size() < 5) f.resize(5);
        const std::string name = f[0];
        const int cmdID = (int)strtol(f[1].c_str(), nullptr, 0);
        const int varID = (int)strtol(f[2].c_str(), nullptr, 0);
        const std::string objID = f[3];
        const std::vector<unsigned char> payload = fromHex(f[4]);

        tcpip::Storage add;
        if (!payload.empty()) add.writePacket(payload);

        try {
            // the whole body of the relay, in four values, payload opaque
            std::lock_guard<std::mutex> lock(libtraci::Connection::getActive().getMutex());
            tcpip::Storage& r = libtraci::Connection::getActive()
                                .doCommand(cmdID, varID, objID, payload.empty() ? nullptr : &add, -1);
            std::vector<unsigned char> tail;
            while (r.valid_pos()) tail.push_back(r.readChar());
            out << name << "|OK|" << toHex(tail) << "\n";
            printf("[probe] %-28s OK  %zu bytes\n", name.c_str(), tail.size());
        } catch (const std::exception& e) {
            out << name << "|EXC|" << e.what() << "\n";
            printf("[probe] %-28s EXC %s\n", name.c_str(), e.what());
        }
    }
    // the connection must still be TrafficLayer's to use afterwards, and the relayed
    // setters must have landed in SUMO - one more step, then compare the state
    // against the same sequence driven by real traci in phase A
    printf("[probe] after relay: ego speed=%.6f lane=%d\n",
           libtraci::Vehicle::getSpeed("ego"), libtraci::Vehicle::getLaneIndex("ego"));
    libtraci::Simulation::step();
    char after[128];
    snprintf(after, sizeof(after), "%.9g,%d",
             libtraci::Vehicle::getSpeed("ego"), libtraci::Vehicle::getLaneIndex("ego"));
    out << "__after__|OK|" << after << "\n";
    out.close();
    printf("[probe] stepped once more, sim time %.2f, after=%s\n",
           libtraci::Simulation::getTime(), after);
    {
        auto res = libtraci::Vehicle::getSubscriptionResults("ego");
        printf("[probe] subscription after relaying: speed=%.6f lane=%d (should match above)\n",
               std::static_pointer_cast<libsumo::TraCIDouble>(res[libsumo::VAR_SPEED])->value,
               (int)std::static_pointer_cast<libsumo::TraCIInt>(res[libsumo::VAR_LANE_INDEX])->value);
    }

    // cost: a relayed getter vs the same getter through libtraci's typed API.
    // The FIXS hop is not in this number - this is the doCommand half only.
    {
        const int N = 2000;
        const std::vector<unsigned char> none;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < N; ++i) {
            tcpip::Storage& r = libtraci::Connection::getActive()
                                .doCommand(0xa4, 0x40, "ego", nullptr, -1);
            while (r.valid_pos()) r.readChar();
        }
        auto t1 = std::chrono::steady_clock::now();
        for (int i = 0; i < N; ++i) (void)libtraci::Vehicle::getSpeed("ego");
        auto t2 = std::chrono::steady_clock::now();
        const double relayed = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
        const double native = std::chrono::duration<double, std::micro>(t2 - t1).count() / N;
        printf("[probe] getSpeed cost over %d calls: relayed %.1f us, libtraci typed %.1f us\n",
               N, relayed, native);
    }

    // can the relay run off the tick thread? libtraci's own domains take
    // Connection::getMutex() around every doCommand, and Connection::simulationStep
    // takes the same mutex, so a relay thread that holds it should serialize against
    // TrafficLayer's stepping instead of corrupting the stream. Measured, not assumed.
    {
        std::atomic<int> relayed{0};
        std::atomic<bool> failed{false};
        std::string err;
        std::thread t([&]() {
            for (int i = 0; i < 300; ++i) {
                try {
                    std::lock_guard<std::mutex> lock(libtraci::Connection::getActive().getMutex());
                    tcpip::Storage& r = libtraci::Connection::getActive()
                                        .doCommand(0xa4, 0x40, "ego", nullptr, -1);
                    while (r.valid_pos()) r.readChar();
                    relayed++;
                } catch (const std::exception& e) { failed = true; err = e.what(); break; }
            }
        });
        int stepped = 0;
        for (int i = 0; i < 300; ++i) {
            try {
                libtraci::Simulation::step();
                (void)libtraci::Vehicle::getSpeed("ego");
                stepped++;
            } catch (const std::exception& e) { failed = true; err = e.what(); break; }
        }
        t.join();
        printf("[probe] concurrent: %d relayed getters against %d steps on another thread -> %s%s\n",
               relayed.load(), stepped, failed ? "FAILED: " : "clean", failed ? err.c_str() : "");
    }

    // why the refuse list is load-bearing: CMD_SIMSTEP fits the four-value contract
    // exactly as well as getSpeed does, so nothing stops a relayed script from
    // advancing the clock behind TrafficLayer's back unless the relay refuses it.
    {
        const double before = libtraci::Simulation::getTime();
        tcpip::Storage add;
        add.writeDouble(0.);  // traci packs CMD_SIMSTEP's argument as a raw double
        tcpip::Storage& r = libtraci::Connection::getActive().doCommand(0x02, -1, "", &add, -1);
        while (r.valid_pos()) r.readChar();
        printf("[probe] relayed CMD_SIMSTEP: sim time %.2f -> %.2f\n",
               before, libtraci::Simulation::getTime());
    }

    libtraci::Simulation::close();
    printf("[probe] closed\n");
    return 0;
}
