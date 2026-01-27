
#ifndef REF_LOC_NODE_INCLUDE_REFLECTOR_MAP_H
#define REF_LOC_NODE_INCLUDE_REFLECTOR_MAP_H
#include <ros/package.h>

#include <map>
#include <string>
#include <vector>

#include "location_option.h"

namespace location {
  constexpr int RefInitPoseStableCountThreshold = 30;

class Reflector {
 public:
  Reflector() : x_(0.), y_(0.), id_(0), count_(1) {xy_deque_.clear();}
  Reflector(double x, double y) : x_(x), y_(y), id_(0), count_(1) {xy_deque_.clear();}
  Reflector(double x, double y, uint32_t id)
      : x_(x), y_(y), id_(id), count_(1) {xy_deque_.clear();}
  Reflector(double x, double y, uint32_t id, uint32_t count)
      : x_(x), y_(y), id_(id), count_(count) {xy_deque_.clear();}
  inline double x() { return x_; }
  inline double y() { return y_; }
  inline uint32_t id() { return id_; }
  inline uint32_t count() { return count_; }

  void setx(double x) { x_ = x; }
  void sety(double y) { y_ = y; }
  void setcount(double count) { count_ = count; }

  void update(double x, double y) {
    x_ = x;
    y_ = y;
  }

  void update(double x, double y, uint32_t count) {
    if (!use_median_filter_) {
      accruex_ += x;
      accruey_ += y;
      x_ = accruex_ / count_;
      y_ = accruey_ / count_;
      count_ = count;

      return;
    }

    if (xy_deque_.size() < 10) {
      xy_deque_.push_back({x, y});
    } else {
      xy_deque_.pop_front();
      xy_deque_.push_back({x, y});
    }
    if(xy_deque_.empty()) return;

    std::vector<float> x_vec, y_vec;
    x_vec.clear();
    y_vec.clear();

    for (auto xy : xy_deque_) {
      x_vec.push_back(xy.first);
      y_vec.push_back(xy.second);
    }
    x_ = median_filter(x_vec);
    y_ = median_filter(y_vec);
    count_ = count;
  }

  float median_filter(std::vector<float> &values) {
    if(values.empty()) return 0.0f;

    const size_t n = values.size();
    std::sort(values.begin(), values.end());

    return values[n / 2];
  }

  void set_use_median_filter(bool use_median_filter) {
    use_median_filter_ = use_median_filter;
  }

  bool get_use_median_filter() {
    return use_median_filter_;
  }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double accruex_ = 0.0;
  double accruey_ = 0.0;
  uint32_t id_ = 0;
  uint32_t count_ = 1;
  std::deque<std::pair<float, float>> xy_deque_;
  bool use_median_filter_ = false;
};

typedef std::vector<Reflector> ReflectorList;

class ReflectorMap {
 public:
  // // 默认构造函数
  // ReflectorMap() = default;

  // // 拷贝构造函数
  // ReflectorMap(const ReflectorMap& other)
  //     : std::vector<Point>(other),  // 拷贝父类部分
  //       dist_m(other.dist_m)        // 拷贝 Eigen 矩阵
  // {}

  // // 拷贝赋值运算符
  // ReflectorMap& operator=(const ReflectorMap& other) {
  //   if (this != &other) {
  //     map_ = other.map_;
  //     trans_map_ = other.trans_map_;
  //   }
  //   return *this;
  // }

  // // 移动构造函数
  // ReflectorMap(ReflectorMap&& other) noexcept
  //     : std::vector<Point>(std::move(other)),
  //       dist_m(std::move(other.dist_m))
  // {}

  // // 移动赋值运算符
  // ReflectorMap& operator=(ReflectorMap&& other) noexcept {
  //     if (this != &other) {
  //         std::vector<Point>::operator=(std::move(other));
  //         dist_m = std::move(other.dist_m);
  //     }
  //     return *this;
  // }

  // // 析构函数
  // ~ReflectorMap() = default; // Eigen 矩阵会自动释放内存
  
  ReflectorMap() {}
  void InitMap();
  void ClearMap() { map_.clear(); }

  std::map<uint32_t, Reflector> GetMap() { return map_;}

  void set_use_median_filter(bool use_median_filter) {
    use_median_filter_ = use_median_filter;
  }

  bool get_use_median_filter() {
    return use_median_filter_;
  }

  void AddMap(ReflectorMap& map) {
    for (auto ref : map.GetMap()) {
      this->Add(ref.second.x(), ref.second.y());
    }
  }

  ReflectorList GetReflectorList() {
    ReflectorList tmp_list;
    if (map_.empty()) return tmp_list;
    for (auto map : map_) {
      Reflector ref(map.second);
      tmp_list.push_back(ref);
    }
    return tmp_list;
  }

  void LoadMapFromFile(std::string file) {
    ROS_INFO("==============Load Reflector Map From File==============");
    map_.clear();
    if (file == "") {
      std::string path =
          ros::package::getPath(PackageName) + std::string("/map/");
      file = path + "reflector_map.xml";
      ROS_INFO("reflector map file path: %s", file.c_str());
    }

    std::time_t last_write_time = boost::filesystem::last_write_time(file);
    ref_map_file_last_write_time_ = static_cast<long>(last_write_time);

    Ptree pt;
    try {
      boost::property_tree::xml_parser::read_xml(
          file, pt, boost::property_tree::xml_parser::trim_whitespace,
          std::locale());
    } catch (std::exception ex) {
      ROS_ERROR_STREAM("can`t read xml file:reflector_map.xml");
      ROS_ERROR_STREAM(ex.what());
    }
    auto& root = pt.get_child("reflectors");
    int cnt = 0;
    for (Ptree::iterator pos = root.begin(); pos != root.end(); ++pos) {
      if (pos->first == "<xmlattr>") continue;
      try {
        cnt++;
        auto element = pos->second;
        auto id = cnt;
        auto px = element.get<std::string>("<xmlattr>.x");
        auto py = element.get<std::string>("<xmlattr>.y");
        auto count = element.get<std::string>("<xmlattr>.count");

        map_.emplace(id,
                     Reflector(atof(px.c_str()), atof(py.c_str()),id,atoi(count.c_str())));
        ROS_INFO_STREAM("load reflector No." << id << " :(" << px << ","
                                             << py << ")");
      } catch (std::exception ex) {
        ROS_ERROR_STREAM("[ReflectorMap]: can`t find the element");
        ROS_ERROR_STREAM(ex.what());
      }
    }
  }


  bool CheckMapFileChanged() {
    std::string file = "";

    if (file == "") {
      std::string path = ros::package::getPath(PackageName) + std::string("/map/");
      file = path + "reflector_map.xml";
    }

    std::time_t update_time = boost::filesystem::last_write_time(file);
    if(ref_map_file_last_write_time_ != update_time) {
        ROS_WARN("---ref map file changed....");
        return true;
    } else {
        return false;
    }
  }

  void SaveNewMapToFile(std::string file) {
    std::string file_backup;
    boost::property_tree::xml_writer_settings<std::string> settings =
        boost::property_tree::xml_writer_make_settings<std::string>('\t', 1, "utf-8");
    if (file == "") {
      std::string path = ros::package::getPath(PackageName) + std::string("/map/");
      file = path + "reflector_map.xml";
      file_backup = path + "reflector_map_backup" + TimeNow() + ".xml";
    }
    // 新建地图：备份老地图
    if (boost::filesystem::exists(file)) {
      boost::filesystem::detail::copy_file(file, file_backup, boost::filesystem::detail::copy_option::overwrite_if_exists);
      boost::filesystem::detail::remove(file);
    }
    // map_file.open(file, std::ios::app);
    // map_file.clear();
    // map_file.close();
    // 新建地图：写入数据
    Ptree pt;
    Ptree guides;
    // guides.put<int>("Version", 1);

    if(!map_.empty()) {
      std::vector<Reflector> ref_vec;
      for (auto ref : map_) { 
        ref_vec.push_back(ref.second);
      }

      std::sort(ref_vec.begin(), ref_vec.end(), [](Reflector& a, Reflector& b){return a.x() < b.x();});
      map_.clear();
      for(int i=0; i<ref_vec.size(); i++){
        map_[i] = ref_vec.at(i);
      }

    }
    
    for (auto ref : map_) {
      if((ref.second.x() == 0.0 && ref.second.y() == 0.0)/* || ref.second.count() < RefInitPoseStableCountThreshold*/) continue;

      auto tree = ToPtree(ref.second);
      guides.add_child("reflector", tree);
    }

    pt.add_child("reflectors", guides);
    boost::property_tree::xml_parser::write_xml(file, pt, std::locale(),
                                                settings);
  }

  void ClearOldTransMap() { 

    std::string path =
          ros::package::getPath(PackageName) + std::string("/map/");
    std::string file = path + "trans_reflector_map.xml";

    // 新建地图：del老地图
    if (boost::filesystem::exists(file)) {
      boost::filesystem::detail::remove(file);
    }
    trans_map_.clear(); 
  }

  void SaveTransMapToFile(std::string file) {
    std::string file_backup;
    boost::property_tree::xml_writer_settings<std::string> settings = boost::property_tree::xml_writer_make_settings<std::string>('\t', 1, "utf-8");
    if (file == "") {
      std::string path = ros::package::getPath(PackageName) + std::string("/map/");
      file = path + "trans_reflector_map.xml";
    }

    // 新建地图：写入数据
    Ptree pt;
    Ptree guides;
    // guides.put<int>("Version", 1);

    if(!trans_map_.empty()) {
      std::vector<Reflector> ref_vec;
      for (auto ref : trans_map_) { 
        ref_vec.push_back(ref.second);
      }

      std::sort(ref_vec.begin(), ref_vec.end(), [](Reflector& a, Reflector& b){return a.x() < b.x();});
      trans_map_.clear();
      for(int i=0; i<ref_vec.size(); i++){
        trans_map_[i] = ref_vec.at(i);
      }

    }

    for (auto ref : trans_map_) {
      auto tree = ToPtree(ref.second);
      guides.add_child("reflector", tree);
    }

    pt.add_child("reflectors", guides);
    boost::property_tree::xml_parser::write_xml(file, pt, std::locale(),
                                                settings);
  }



  void Add_Trans_Map(double x, double y,int id, int count) {
    // uint32_t id = 1;
    // if (trans_map_.empty()) {
    //   id = 1;
    // } else {
    //   id = trans_map_.size() + 1;
    // }
    ROS_DEBUG_STREAM("id:" << id << ",count:" << count);

    Reflector reflector(x, y, id, count);

    trans_map_[id] = reflector;
  }

  void AppendMapToFile(std::string file) {
    std::string file_backup;
    boost::property_tree::xml_writer_settings<std::string> settings = boost::property_tree::xml_writer_make_settings<std::string>('\t', 1, "utf-8");
    if (file == "") {
      std::string path = ros::package::getPath(PackageName) + std::string("/map/");
      file = path + "reflector_map.xml";
      file_backup = path + "reflector_map_backup" + TimeNow() + ".xml";
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

    // 新建地图：备份老地图
    if (boost::filesystem::exists(file)) {
      boost::filesystem::detail::copy_file(file, file_backup, boost::filesystem::detail::copy_option::overwrite_if_exists);
      boost::filesystem::detail::remove(file);
    }

    auto& root = pt.get_child("reflectors");
    ReflectorList ref_list;
    int cnt = 0;
    for (Ptree::iterator pos = root.begin(); pos != root.end(); ++pos) {
      if (pos->first == "<xmlattr>") continue;
      cnt++;
      auto element = pos->second;
      auto id = element.get<std::string>("<xmlattr>.id");
      auto x = element.get<std::string>("<xmlattr>.x");
      auto y = element.get<std::string>("<xmlattr>.y");
      ref_list.push_back(
          Reflector(atof(x.c_str()), atof(y.c_str()), atoi(id.c_str())));
    }
    for (auto ref : map_) {
      bool same_ref = false;
      for (auto ref_check : ref_list) {
        same_ref = std::hypot(ref_check.x() - ref.second.x(),
                              ref_check.y() - ref.second.y()) < DistanceThresholdBetweenRefs2;
        if (same_ref) break;
      }
      if ((ref.second.x() == 0.0 && ref.second.y() == 0.0) || same_ref /*||
          ref.second.count() < RefInitPoseStableCountThreshold*/)
        continue;
      uint32_t id = root.size() + 1;
      Reflector tmp_ref(ref.second.x(), ref.second.y(), id, ref.second.count());
      auto tree = ToPtree(tmp_ref);
      root.add_child("reflector", tree);
    }
    boost::property_tree::xml_parser::write_xml(file, pt, std::locale(),
                                                settings);
  }

  void Add(Reflector ref) { map_[ref.id()] = ref; }

  int Add(double x, double y) {
    uint32_t id = 1;
    if (map_.empty()) {
      id = 1;
    } else {
      id = map_.size() + 1;
    }
    Reflector reflector(x, y, id);
    map_[id] = reflector;

    return id;
  }

  int Add(double x, double y, uint32_t no) {
    uint32_t id = no;

    if(map_.count(id) == 0) {
      Reflector reflector(x, y, id);
      map_[id] = reflector;
    }else {
      uint32_t count = map_[id].count();
      count += 1;

      map_[id].set_use_median_filter(use_median_filter_);
      int count_threshold = 3;
      if (use_median_filter_) {
        count_threshold = RefInitPoseStableCountThreshold;
      }
      
      if (count < count_threshold) {
        map_[id].update(x, y, count);
      } else {
        map_[id].setcount(count);
      }
    }

    return id;
  }

  void Modify(uint32_t id, double x, double y) {
    if (map_.count(id) != 0) {
      map_[id].update(x, y);
    }
  }

  void Update(uint32_t no) {
    uint32_t id = no;
    uint32_t count = map_[id].count();
    count += 1;
    map_[id].setcount(count);
  }

  void DeleteReflector(uint32_t id) {
    if (map_.count(id) != 0) {
      map_.erase(id);
    }
  }

  PointSet ToPointSet() {
    PointSet pts;
    for (auto ref : map_) {
      Point pt(ref.second.x(), ref.second.y());
      pts.push_back(pt);
    }
    return pts;
  }

  std::vector<int> ToIds() {
    std::vector<int> ids;
    for (auto ref : map_) {
      ids.push_back(ref.second.id());
    }
    return ids;
  }

 private:
  std::map<uint32_t, Reflector> map_;
  std::map<uint32_t, Reflector> trans_map_;
  std::ofstream map_file;
  long ref_map_file_last_write_time_;
  bool use_median_filter_ = false;

  Ptree ToPtree(Reflector ref) {
    Ptree data_pt;
    std::string type_str(std::to_string(0));

    data_pt.put("<xmlattr>.id", std::to_string(ref.id()));
    data_pt.put("<xmlattr>.x", std::to_string(ref.x()));
    data_pt.put("<xmlattr>.y", std::to_string(ref.y()));
    data_pt.put("<xmlattr>.count", std::to_string(ref.count()));
    data_pt.put("<xmlattr>.type", type_str);

    return data_pt;
  }
};

}  // namespace location
#endif