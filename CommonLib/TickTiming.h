#pragma once

#include <chrono>
#include <cstdio>
#include <map>
#include <set>

namespace fixs {

// Wall time of each TrafficLayer tick, split into its phases and per client, with relayed TraCI
// calls counted per client. Only ticks that exchange with the clients are counted.
class TickTiming {
public:
	using clock = std::chrono::steady_clock;
	enum Phase { Step, ReadSim, Clients, WriteSim, Other, kPhases };

	bool report = false;          // SimulationSetup.TimingReport: print every reportPeriod of sim time
	double reportPeriod = 10.0;   // sim seconds
	double warnShare = 0.10;      // relayed calls above this share of the tick warn once per client

	void beginTick() {
		lap_ = clock::now();
		open_ = true;
		for (double& v : cur_) v = 0;
		curClients_.clear();
	}

	// Charges the time since the previous lap to phase p.
	void lap(Phase p) {
		if (!open_) return;
		const auto t = clock::now();
		cur_[p] += ms(lap_, t);
		lap_ = t;
	}

	void client(int port, clock::time_point start, double relayMs, long long relayCalls) {
		if (!open_) return;
		Acc& a = curClients_[port];
		a.ms += ms(start, clock::now());
		a.relayMs += relayMs;
		a.relayCalls += (double)relayCalls;
	}

	void endTick(double simTime) {
		if (!open_) return;
		lap(Other);
		open_ = false;
		const auto now = clock::now();
		add(win_, simTime, now);
		add(run_, simTime, now);
		if (win_.t1 - win_.t0 >= reportPeriod - 1e-6) {
			warn(win_);
			if (report) print("[timing]", win_);
			win_ = Window();
		}
	}

	void finish() {
		if (win_.ticks > 0) warn(win_);
		if (report && run_.ticks > 0) print("[timing] whole run:", run_);
	}

private:
	struct Acc { double ms = 0, relayMs = 0, relayCalls = 0; };
	struct Window {
		long long ticks = 0;
		double t0 = 0, t1 = 0, phase[kPhases] = {};
		std::map<int, Acc> clients;
		clock::time_point wall0, wall1;
	};

	static double ms(clock::time_point a, clock::time_point b) {
		return std::chrono::duration<double, std::milli>(b - a).count();
	}

	void add(Window& w, double simTime, clock::time_point now) {
		if (w.ticks == 0) {
			w.t0 = simTime;
			w.wall0 = now;
		}
		w.ticks++;
		w.t1 = simTime;
		w.wall1 = now;
		for (int p = 0; p < kPhases; p++) w.phase[p] += cur_[p];
		for (auto& c : curClients_) {
			Acc& a = w.clients[c.first];
			a.ms += c.second.ms;
			a.relayMs += c.second.relayMs;
			a.relayCalls += c.second.relayCalls;
		}
	}

	static double total(const Window& w) {
		double s = 0;
		for (double v : w.phase) s += v;
		return s;
	}

	void warn(const Window& w) {
		const double tick = total(w) / w.ticks;
		for (auto& c : w.clients) {
			const double relay = c.second.relayMs / w.ticks;
			if (warned_.count(c.first) || tick <= 0 || relay < warnShare * tick) continue;
			warned_.insert(c.first);
			printf("WARNING: TrafficLayer spent %.1f ms of each %.1f ms tick (%.0f%%) answering %.0f "
			       "relayed TraCI calls per tick from the client on port %d (t=%.1f-%.1f), and that "
			       "client waits on each one. Every relayed call is a round trip through TrafficLayer "
			       "to SUMO; per-tick state is cheaper read from the FIXS feed (fixs.vehicle, "
			       "fixs.trafficlight). SimulationSetup.TimingReport: true prints the full breakdown.\n",
			       relay, tick, 100.0 * relay / tick, c.second.relayCalls / w.ticks, c.first, w.t0, w.t1);
		}
	}

	void print(const char* tag, const Window& w) {
		const double n = (double)w.ticks;
		const double wallS = ms(w.wall0, w.wall1) / 1000.0;
		printf("%s t=%.1f-%.1f %.1f ms/tick, %.2fx real time | SUMO step %.1f | read SUMO %.1f | "
		       "clients %.1f | write SUMO %.1f | other %.1f",
		       tag, w.t0, w.t1, total(w) / n, wallS > 0 ? (w.t1 - w.t0) / wallS : 0.0,
		       w.phase[Step] / n, w.phase[ReadSim] / n, w.phase[Clients] / n,
		       w.phase[WriteSim] / n, w.phase[Other] / n);
		for (auto& c : w.clients) {
			printf(" | port %d %.1f", c.first, c.second.ms / n);
			if (c.second.relayCalls > 0) {
				printf(" (relay %.1f ms, %.1f calls)", c.second.relayMs / n, c.second.relayCalls / n);
			}
		}
		printf("\n");
	}

	bool open_ = false;
	clock::time_point lap_;
	double cur_[kPhases] = {};
	std::map<int, Acc> curClients_;
	Window win_, run_;
	std::set<int> warned_;
};

}  // namespace fixs
