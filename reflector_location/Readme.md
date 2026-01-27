# reflector_location
## 这个是程晓辉写的反光柱定位的ROS程序  
### 其中`反光板定位.md` 文件是程晓辉写的一些说明，后续有什么补充，建议写在这个markdown中  
### 说明：config中，`reflector_map.xml`文件是反光柱地图，`reflector_location_param.xml`是反光柱定位的配置文件  
### 编译:`catkin_make`  
### 运行 `roslaunch reflector_location reflector_location.launch`  
### `reflector_location`话题是计算出来的里程计话题（计算出来的定位数据），`mapping_order`话题是设置反光柱slam的模式的，0代表纯定位模式，1代表建图模式，2代表增补模式，99代表结束建图，节点发布map->base_link的tf，外参设置在reflector_location_param.xml文件中    