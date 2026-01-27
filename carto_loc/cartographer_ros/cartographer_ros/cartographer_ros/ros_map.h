/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
 #define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
 
 #include <string>
 
 #include "Eigen/Core"
 #include "cartographer/io/file_writer.h"
 #include "cartographer/io/image.h"
 #include "cartographer/mapping/2d/map_limits.h"
 #include "cartographer/common/config_file_operator.h"
 
 #define UNKNOWN_VALUE                                       (128)
 
 namespace cartographer_ros {
 
 // SlamCommon::CConfigFileOperator *m_config_oper =new SlamCommon::CConfigFileOperator("/home/yang/work_space/final_carto_ros/") ;
 
 // Write 'image' as a pgm into 'file_writer'. The resolution is used in the
 // comment only'
 void WritePgm(const ::cartographer::io::Image& image, const double resolution,
               ::cartographer::io::FileWriter* file_writer);
 void WritePgm(const std::string &map_name,const ::cartographer::io::Image& image, const double resolution,
               ::cartographer::io::FileWriter* file_writer);
 
 // Write the corresponding yaml into 'file_writer'.
 void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
                const std::string& pgm_filename,
                ::cartographer::io::FileWriter* file_writer);
 
 template <typename T>
 std::string NumToString(const T num, int decplaces = 3)
 {
     std::ostringstream oss;
     //std::fixed means keep decplaces behind decimal point
     oss << std::fixed << std::setprecision(decplaces) << num;
     return oss.str();
 }
 
 // Write the corresponding xml into 'file_writer'.
 // void WriteXml(const std::string &map_name,
 // 	const std::string& xml_name, const double resolution ,const Eigen::Vector2d origin,SlamCommon::CConfigFileOperator *m_config_oper);
 
 void WriteXml(const std::string &map_name,
     const std::string& xml_name, const double resolution ,const Eigen::Vector2d origin);
 
 }  // namespace cartographer_ros
 
 #endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H