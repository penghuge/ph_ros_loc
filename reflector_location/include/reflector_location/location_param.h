

#ifndef REF_LOC_NODE_INCLUDE_LOCATION_PARAM_H
#define REF_LOC_NODE_INCLUDE_LOCATION_PARAM_H
#include "reflector_location/location_option.h"

namespace location {

class LocationConfigParam {
 public:
  LocationConfigParam(const LocationConfigParam&) = delete;
  LocationConfigParam& operator=(const LocationConfigParam&) = delete;
  ~LocationConfigParam() {}
  static LocationConfigParam& Instance() {
    static LocationConfigParam instance;
    return instance;
  }
  LocationOption GetOption() { return option_; }

  bool LoadParamFromFile(std::string file) {
    ROS_INFO("==============Load Location Parameter From File==============");
    if (file == "") {
      std::string path =
          ros::package::getPath(PackageName) + std::string("/config/");
      file = path + "reflector_location_param.xml";
    }
    Ptree pt;
    try {
      boost::property_tree::xml_parser::read_xml(
          file, pt, boost::property_tree::xml_parser::trim_whitespace,
          std::locale());
    } catch (std::exception ex) {
      ROS_ERROR_STREAM("can`t read xml file");
      ROS_ERROR_STREAM(ex.what());
    }
    try {
      auto& params = pt.get_child("params");
      {
        ROS_INFO("----------------- node option --------------------");
        auto node_option = params.get_child("node_option");
        option_.node_option.use_imu = node_option.get<bool>("use_imu");
        ROS_WARN_STREAM("use_imu: " << option_.node_option.use_imu);
        option_.node_option.use_odom = node_option.get<bool>("use_odom");
        ROS_WARN_STREAM("use_odom: " << option_.node_option.use_odom);

        option_.node_option.use_location_map =
            node_option.get<bool>("use_location_map");

        ROS_WARN_STREAM("use_location_map: " << option_.node_option.use_location_map);
        option_.node_option.use_location_config =
            node_option.get<bool>("use_location_config");
        ROS_WARN_STREAM(
            "use_location_config: " << option_.node_option.use_location_config);
      }

      {
        ROS_INFO("------------- system model nosie -----------------");
        auto sys_model_nosie = params.get_child("sys_model_nosie");
        option_.sys_model_nosie.x = sys_model_nosie.get<double>("<xmlattr>.x");
        ROS_WARN_STREAM("sys_model_nosie.x: " << option_.sys_model_nosie.x);
        option_.sys_model_nosie.y = sys_model_nosie.get<double>("<xmlattr>.y");
        ROS_WARN_STREAM("sys_model_nosie.y: " << option_.sys_model_nosie.y);
        option_.sys_model_nosie.theta =
            sys_model_nosie.get<double>("<xmlattr>.theta");
        ROS_WARN_STREAM(
            "sys_model_nosie.theta: " << option_.sys_model_nosie.theta);
      }

      {
        ROS_INFO("--------------- reflector option -----------------");
        auto sys_model_nosie =
            params.get_child("reflector_option.sys_model_nosie");
        option_.ref_option.sys_model_nosie.x =
            sys_model_nosie.get<double>("<xmlattr>.x");
        ROS_WARN_STREAM(
            "sys_model_nosie.x: " << option_.ref_option.sys_model_nosie.x);
        option_.ref_option.sys_model_nosie.y =
            sys_model_nosie.get<double>("<xmlattr>.y");
        ROS_WARN_STREAM(
            "sys_model_nosie.y: " << option_.ref_option.sys_model_nosie.y);
        option_.ref_option.sys_model_nosie.theta =
            sys_model_nosie.get<double>("<xmlattr>.theta");
        ROS_WARN_STREAM("sys_model_nosie.theta: "
                        << option_.ref_option.sys_model_nosie.theta);

        auto measure_nosie = params.get_child("reflector_option.measure_nosie");
        option_.ref_option.measure_nosie.x =
            measure_nosie.get<double>("<xmlattr>.x");
        ROS_WARN_STREAM(
            "measure_nosie.x: " << option_.ref_option.measure_nosie.x);
        option_.ref_option.measure_nosie.y =
            measure_nosie.get<double>("<xmlattr>.y");
        ROS_WARN_STREAM(
            "measure_nosie.y: " << option_.ref_option.measure_nosie.y);
        option_.ref_option.measure_nosie.theta =
            measure_nosie.get<double>("<xmlattr>.theta");
        ROS_WARN_STREAM(
            "measure_nosie.theta: " << option_.ref_option.measure_nosie.theta);
      }

      if (option_.node_option.use_odom) {
        ROS_INFO("----------------- odom option --------------------");
        auto measure_nosie = params.get_child("odom_option.measure_nosie");
        option_.odom_option.x = measure_nosie.get<double>("<xmlattr>.x");
        ROS_WARN_STREAM("measure_nosie.x: " << option_.odom_option.x);
        option_.odom_option.y = measure_nosie.get<double>("<xmlattr>.y");
        ROS_WARN_STREAM("measure_nosie.y: " << option_.odom_option.y);
        option_.odom_option.theta =
            measure_nosie.get<double>("<xmlattr>.theta");
        ROS_WARN_STREAM("measure_nosie.theta: " << option_.odom_option.theta);
      }

    } catch (std::exception ex) {
      ROS_ERROR_STREAM("Parameter file have error！！");
      ROS_ERROR_STREAM(ex.what());
      return false;
    }
    return true;
  }

  void UpdateParam() {}

  void SaveParamToFile(std::string file) {
    std::string file_backup;
    if (file == "") {
      std::string path = ros::package::getPath(PackageName) + std::string("/");
      file = path + "reflector_location_param.xml";
      file_backup = path + "reflector_location_param_backup_" + TimeNow() + ".xml";
    }
    // 备份老参数文件
    if (boost::filesystem::exists(file)) {
      boost::filesystem::detail::copy_file(
          file, file_backup,
          boost::filesystem::detail::copy_option::overwrite_if_exists);
      boost::filesystem::detail::remove(file);
    }
  }

 private:
  LocationOption option_;
  LocationConfigParam() {}
};

}  // namespace location
#endif