// Provides process-local degeneracy flag implementation in core library
// so that any binary linking libcartographer resolves these symbols.

#include <atomic>
#include <mutex>
#include "glog/logging.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/internal/2d/scan_matching/degeneracy_flag.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {
// Process-local flag; default false. Updated by higher layers (e.g., ROS node).
static std::atomic<bool> g_degeneracy_detected{false};
}  // namespace

void SetDegeneracyDetected(bool v) {
  g_degeneracy_detected.store(v, std::memory_order_relaxed);
}

bool GetDegeneracyDetected() {
  return g_degeneracy_detected.load(std::memory_order_relaxed);
}

// --- Shared local/global storage for latest localization and external odom ---
namespace {
std::mutex g_loc_mutex;
cartographer::transform::Rigid2d g_latest_loc = cartographer::transform::Rigid2d::Identity();
double g_latest_loc_score = 0.0;
bool g_latest_loc_valid = false;

std::mutex g_odom_mutex;
double g_odom_x = 0.0, g_odom_y = 0.0, g_odom_theta = 0.0;
bool g_use_external_odom = false;
bool g_have_external_odom = false;
}  // namespace

void SetLatestLocalization2D(const cartographer::transform::Rigid2d& pose, double score) {
  std::lock_guard<std::mutex> lk(g_loc_mutex);
  g_latest_loc = pose;
  g_latest_loc_score = score;
  g_latest_loc_valid = true;
}

bool GetLatestLocalization2D(cartographer::transform::Rigid2d* pose, double* score) {
  std::lock_guard<std::mutex> lk(g_loc_mutex);
  if (!g_latest_loc_valid) return false;
  if (pose) *pose = g_latest_loc;
  if (score) *score = g_latest_loc_score;
  return true;
}

bool GetLatestLocalizationXYT(double* x, double* y, double* theta, double* score) {
  std::lock_guard<std::mutex> lk(g_loc_mutex);
  if (!g_latest_loc_valid) return false;
  if (x) *x = g_latest_loc.translation().x();
  if (y) *y = g_latest_loc.translation().y();
  if (theta) *theta = g_latest_loc.rotation().angle();
  if (score) *score = g_latest_loc_score;
  return true;
}

void SetExternalOdometryPose2D(double x, double y, double theta) {
  std::lock_guard<std::mutex> lk(g_odom_mutex);
  g_odom_x = x; g_odom_y = y; g_odom_theta = theta;
  g_have_external_odom = true;
}

bool GetExternalOdometryPose2D(double* x, double* y, double* theta) {
  std::lock_guard<std::mutex> lk(g_odom_mutex);
  if (!g_have_external_odom) return false;
  if (x) *x = g_odom_x; if (y) *y = g_odom_y; if (theta) *theta = g_odom_theta;
  return true;
}

void SetUseExternalOdom(bool v) {
  std::lock_guard<std::mutex> lk(g_odom_mutex);
  if (g_use_external_odom != v) {
    g_use_external_odom = v;
    LOG(INFO) << "[cartographer_core] SetUseExternalOdom=" << (v ? "true" : "false");
  } else {
    VLOG(2) << "[cartographer_core] SetUseExternalOdom called with same value="
            << (v ? "true" : "false");
  }
}

bool GetUseExternalOdom() {
  std::lock_guard<std::mutex> lk(g_odom_mutex);
  LOG(INFO) << "[cartographer_core] GetUseExternalOdom=" << (g_use_external_odom ? "true" : "false");
  return g_use_external_odom;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
