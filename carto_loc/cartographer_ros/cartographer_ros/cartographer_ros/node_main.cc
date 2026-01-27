#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <std_srvs/Trigger.h>

// 命令行标志定义
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(log_directory, "",
              "First directory in which configuration files are searched.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

SlamCommon::CConfigFileOperator *m_config_operator = nullptr;
SlamCommon::CFileInterface *m_file_interface = nullptr;

namespace cartographer_ros {
namespace {
void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  constexpr char AcquireFilePoseServiceName[] = "acquire_file_pose";
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  int match_mode = 1;
  ros::NodeHandle nh;
  
  ros::ServiceClient acquire_file_pose_client = nh.serviceClient<std_srvs::Trigger>(AcquireFilePoseServiceName);

  nh.param("match_mode", match_mode, 1);

  std::tie(node_options, trajectory_options) =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  m_file_interface->SetPureLocalization(match_mode);
  auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(m_file_interface, m_config_operator, node_options.map_builder_options);

  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics, m_file_interface);
  node.SetConfigurationDirectoryAndName(FLAGS_configuration_directory, FLAGS_configuration_basename);
  
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  if(match_mode == 4) match_mode = 3;
  if(match_mode == 1 || match_mode == 3){
    ros::service::waitForService(AcquireFilePoseServiceName);
    std_srvs::Trigger srv;
    srv.request = {};
    acquire_file_pose_client.call(srv);
    while (!srv.response.success){
      acquire_file_pose_client.call(srv);
      ROS_ERROR("Carto call %s failed, get file_pose: %s", AcquireFilePoseServiceName, srv.response.message.c_str());
      usleep(10000);
    }

    cartographer::transform::Rigid3d file_pose = cartographer::transform::Rigid3d({0.0, 0.0, 0.0}, Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
    if(srv.response.success){
      ROS_INFO("Carto call %s success, get file_pose: %s", AcquireFilePoseServiceName, srv.response.message.c_str());

      std::stringstream ss(srv.response.message);
      double file_pose_x, file_pose_y, file_pose_theta;
      ss >> file_pose_x >> file_pose_y >> file_pose_theta;

      file_pose = cartographer::transform::Rigid3d({file_pose_x, file_pose_y, 0.0}, Eigen::AngleAxisd(file_pose_theta, Eigen::Vector3d::UnitZ()));
    }
                                
  //   ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
  //   *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(file_pose);

  //   *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  
  ::ros::spin();
  node.FinishAllTrajectories();
  node.RunFinalOptimization();
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  //init logging
  std::string log_file_path = FLAGS_log_directory + DOCS_LOG_ID;

  LOG_INIT(argv[0], FLAGS_log_directory, FLAGS_log_directory, log_file_path);
  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  m_config_operator = new SlamCommon::CConfigFileOperator(FLAGS_log_directory);
  m_file_interface = new SlamCommon::CFileInterface(FLAGS_log_directory, m_config_operator);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();

  ::ros::shutdown();
}
