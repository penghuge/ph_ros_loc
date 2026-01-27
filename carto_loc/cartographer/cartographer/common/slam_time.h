#ifndef SLAM_TIME_H_
#define SLAM_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>
#include <sstream>

namespace SlamCommon
{
const int64_t kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock
{
typedef int64_t rep;
typedef std::ratio<1, 10000000> period;
typedef std::chrono::duration<rep, period> duration;
typedef std::chrono::time_point<UniversalTimeScaleClock> time_point;
static const bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
typedef UniversalTimeScaleClock::duration Duration;
typedef UniversalTimeScaleClock::time_point Time;

// Convenience functions to create common::Durations.
inline Duration FromSeconds(const double seconds)
{
	return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}
inline Duration FromMilliseconds(const int64_t milliseconds)
{
	return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(milliseconds));
}
inline Duration FromMicroseconds(const int64_t microseconds)
{
	return std::chrono::duration_cast<Duration>(std::chrono::microseconds(microseconds));
}
// Returns the given duration in seconds.
inline double ToSeconds(const Duration duration)
{
	return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}
// Creates a time from a Universal Time Scale.
inline Time FromUniversal(const int64_t ticks)
{
	return Time(Duration(ticks));
}
// Outputs the Universal Time Scale timestamp for a given Time.
inline int64_t ToUniversal(const Time time)
{
	return time.time_since_epoch().count();
}
// For logging and unit tests, outputs the timestamp integer.
inline std::ostream& operator<<(std::ostream& os, const Time time)
{
	std::stringstream ss;
	ss << ToUniversal(time);
	os << ss.str();
	return os;
}
/**********************************************************************************
function: return current time as the format of Time, unit is ms
**********************************************************************************/
inline Time TimeNow()
{
	std::chrono::time_point<std::chrono::steady_clock, Duration> now
		= std::chrono::time_point_cast<Duration>(std::chrono::steady_clock::now());
	return FromUniversal(now.time_since_epoch().count());
}

}  // namespace SlamCommon

#endif  // SLAM_TIME_H_
