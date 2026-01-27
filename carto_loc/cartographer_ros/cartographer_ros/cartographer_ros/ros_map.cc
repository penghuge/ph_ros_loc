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

 #include "cartographer_ros/ros_map.h"

 #include "absl/strings/str_cat.h"
 
 SlamCommon::CConfigFileOperator *m_config_oper = new SlamCommon::CConfigFileOperator("~/");
 
 namespace cartographer_ros {
 
 void WritePgm(const ::cartographer::io::Image& image, const double resolution,
               ::cartographer::io::FileWriter* file_writer) {
   const std::string header =
       absl::StrCat("P5\n# Cartographer map; ", resolution, " m/pixel\n",
                    image.width(), " ", image.height(), "\n255\n");
   file_writer->Write(header.data(), header.size());
   for (int y = 0; y < image.height(); ++y) {
     for (int x = 0; x < image.width(); ++x) {
       const char color = image.GetPixel(x, y)[0];
       file_writer->Write(&color, 1);
     }
   }
 }
 
 void WritePgm(const std::string &map_name,const ::cartographer::io::Image& image, const double resolution,
               ::cartographer::io::FileWriter* file_writer){
   const std::string header =
       // absl::StrCat("P5\n# CSG map: ",map_name,", resolution: ", resolution, " m/pixel\n",
       absl::StrCat("P5\n# CSG map: ",map_name,", resolution: 0.050 m/pixel\n",
                    image.width(), " ", image.height(), "\n255\n");
   file_writer->Write(header.data(), header.size());
   for (int y = 0; y < image.height(); ++y) {
     for (int x = 0; x < image.width(); ++x) {
       unsigned char ucolor = image.GetPixel(x, y)[0];
       char color=ucolor;
       float p=(255-ucolor)/255.0;
       if(p<0.196)
       {
         color=255;
       }
       else if(p>0.65)
       {
         color=0;
       }
       else
       {
         color=UNKNOWN_VALUE;
       }
       file_writer->Write(&color, 1);
     }
   }
 }
 
 void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
                const std::string& pgm_filename,
                ::cartographer::io::FileWriter* file_writer) {
   // Magic constants taken directly from ros map_saver code:
   // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
   const std::string output = absl::StrCat(
       "image: ", pgm_filename, "\n", "resolution: ", resolution, "\n",
       "origin: [", origin.x(), ", ", origin.y(),
       ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
   file_writer->Write(output.data(), output.size());
 }
 
 
 
 void WriteXml(const std::string &map_name,
   const std::string& xml_name, const double resolution ,const Eigen::Vector2d origin)
 {
     // LOG_INFO("Saving map to %s..", xml_name.c_str());
     std::ofstream xml_file(xml_name, std::ios::out | std::ios::binary);
 
     m_config_oper->InitConfigFileOperatorForWriting();
 
     m_config_oper->SetParam("image", map_name);
     m_config_oper->SetParam("resolution", NumToString(resolution, 3));
     //origin in current map
     m_config_oper->SetParam("origin_pose_x", NumToString(origin.x(), 4));
     m_config_oper->SetParam("origin_pose_y", NumToString(origin.y(), 4));
     m_config_oper->SetParam("origin_pose_theta", NumToString(0.0, 4));
 
     m_config_oper->SetParam("occupied_threash", NumToString(0.51));
     m_config_oper->SetParam("free_threash", NumToString(0.49));
     m_config_oper->SetParam("negate", false);
 
     m_config_oper->WriteXmlFile(xml_name);//write to xml
     m_config_oper->UninitConfigFileOperator();
 
   xml_file.close();
   if((m_config_oper) != nullptr){delete (m_config_oper); (m_config_oper) = nullptr;}
 //     CHECK(xml_file) << "Writing " << xml_name << " failed.");
 }
 
 
 
 }  // namespace cartographer_ros