#ifndef PERFORMANCE_TIMER_H
#define PERFORMANCE_TIMER_H

#include <chrono>
#include <string>
#include <fstream>
#include <unordered_map>
#include <iomanip>
#include <ctime>
#include <cstdarg>

// Toggle performance timing
// Define ENABLE_PERF_TIMING before including this header to enable timing
// (see VirEnvHelper.cpp for example)

#ifdef ENABLE_PERF_TIMING

class PerformanceTimer {
private:
	// File stream for logging (static = shared)
	static std::ofstream perfLogFile;
	static std::string perfLogPath;

	// Tic/toc storage (map label -> start time)
	static std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> ticMap;

public:
	// Initialize performance logging (call once at startup)
	static void init(const std::string& logPath = "RealSimPerf.log") {
		if (!perfLogFile.is_open()) {
			perfLogPath = logPath;
			// Use std::ios::out to truncate file (reset on each init)
			perfLogFile.open(perfLogPath, std::ios::out | std::ios::trunc);

			// Write session header with timestamp
			if (perfLogFile.is_open()) {
				auto now = std::chrono::system_clock::now();
				std::time_t now_c = std::chrono::system_clock::to_time_t(now);
				perfLogFile << "=============================================" << std::endl;
				perfLogFile << "Performance Log Session: " << std::ctime(&now_c);
				perfLogFile << "=============================================" << std::endl;
				perfLogFile.flush();
			}
		}
	}

	// Start timing
	static void tic(const std::string& label) {
		ticMap[label] = std::chrono::high_resolution_clock::now();
	}

	// Stop timing and log
	static void toc(const std::string& label) {
		auto it = ticMap.find(label);
		if (it == ticMap.end()) return;

		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> elapsed = end_time - it->second;

		// Write to performance log file with timestamp
		if (perfLogFile.is_open()) {
			auto now = std::chrono::system_clock::now();
			std::time_t now_c = std::chrono::system_clock::to_time_t(now);
			auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch()) % 1000;

			struct tm timeinfo;
#ifdef _WIN32
			localtime_s(&timeinfo, &now_c);
#else
			localtime_r(&now_c, &timeinfo);
#endif
			char timestamp[64];
			std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);

			perfLogFile << "[" << timestamp << "." << std::setfill('0') << std::setw(3) << now_ms.count() << "] "
				<< "[PERF] " << label << ": "
				<< std::fixed << std::setprecision(3)
				<< elapsed.count() << " ms" << std::endl;
			perfLogFile.flush();
		}

		ticMap.erase(it);
	}

	// Log diagnostic message (printf-style)
	static void logf(const char* format, ...) {
		if (perfLogFile.is_open()) {
			char buffer[512];
			va_list args;
			va_start(args, format);
			vsnprintf(buffer, sizeof(buffer), format, args);
			va_end(args);

			auto now = std::chrono::system_clock::now();
			std::time_t now_c = std::chrono::system_clock::to_time_t(now);
			auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch()) % 1000;

			struct tm timeinfo;
#ifdef _WIN32
			localtime_s(&timeinfo, &now_c);
#else
			localtime_r(&now_c, &timeinfo);
#endif
			char timestamp[64];
			std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);

			perfLogFile << "[" << timestamp << "." << std::setfill('0') << std::setw(3) << now_ms.count() << "] "
				<< "[DIAG] " << buffer;
			perfLogFile.flush();
		}
	}

	// Cleanup (call at shutdown)
	static void shutdown() {
		if (perfLogFile.is_open()) {
			perfLogFile.close();
		}
		ticMap.clear();
	}
};

// Static member initialization
std::ofstream PerformanceTimer::perfLogFile;
std::string PerformanceTimer::perfLogPath;
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> PerformanceTimer::ticMap;

// Convenience macros
#define PERF_INIT(logPath) PerformanceTimer::init(logPath)
#define PERF_SHUTDOWN() PerformanceTimer::shutdown()
#define PERF_TIC(name) PerformanceTimer::tic(name)
#define PERF_TOC(name) PerformanceTimer::toc(name)
#define PERF_LOG(...) PerformanceTimer::logf(__VA_ARGS__)

#else  // ENABLE_PERF_TIMING not defined

// No-op macros when disabled (zero overhead)
#define PERF_INIT(logPath)
#define PERF_SHUTDOWN()
#define PERF_TIC(name)
#define PERF_TOC(name)
#define PERF_LOG(...)

#endif  // ENABLE_PERF_TIMING

#endif  // PERFORMANCE_TIMER_H
