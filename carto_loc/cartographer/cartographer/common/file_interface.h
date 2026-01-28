#ifndef FILE_INTERFACE_H_
#define FILE_INTERFACE_H_

#include <fstream>
#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#endif
#include "glog/logging.h"
#include "format_transform.h"
#include "config_file_operator.h"
#include "agv_data.h"
#include "agv_error_code.h"
#include <mutex>
#include <atomic>
#include <cartographer/mapping/2d/submap_2d.h>
#include "cartographer/carto_localization/scan_matching_2d/precomputation.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace SlamCommon
{
class CFileInterface
{
public:
    typedef std::map<std::string, OccupancyGrid*> DefMapDataMap;

	CFileInterface(const std::string &cur_path, CConfigFileOperator *config_operator);
	~CFileInterface();

    /*********************************************************************
    function: get Docs directory
    *********************************************************************/
    std::string GetDocsPath()
    {
        return m_cur_path + DIR_DOCS_ID + DIR_FLAG;
    }
    std::string GetCurrentPath()
    {
        return m_cur_path;
    }
    //penghu 24/5/09
    void SaveExpandMapData(const std::string& map_name, const OccupancyGrid &occupancy_grid);
    //penghu 24/5/09
    bool ReadExpandMapFromPgm(const std::string &map_name, OccupancyGrid *occupancy_grid);
    //penghu 24/5/09
    OccupancyGrid *GetExpandGrid(const std::string &map_name);
	void SaveMapData(const std::string& map_name, const OccupancyGrid &occupancy_grid);
    void SetNaviType(bool autorunState)
    {
        autorunState_.store(autorunState);
        LOG_DEBUG_THROTTLE(1.0, "SetNaviType = %d", autorunState_.load());
    }
    //penghu 24/5/19
    void SetVel_NaviType(const double &agv_vel, const double &agv_vel_y, const double &agv_vel_w, const bool &autorunState)
    {
        vel_.store(agv_vel);
        vel_y_.store(agv_vel_y);
        vel_w_.store(agv_vel_w);
        autorunState_.store(autorunState);
        LOG_DEBUG_THROTTLE(1.0, "ningde_code_autorunState_set = %d, = %f", autorunState_.load(), vel_.load());
    }
    //penghu 24/6/26
    bool Get_NaviType()
    {
        LOG_DEBUG_THROTTLE(1.0, "ningde_code_ningde_code_autorunState_get = %d", autorunState_.load());
        return autorunState_.load();
    }
    //penghu 24/5/19
    double GetVel()
    {
        LOG_DEBUG_THROTTLE(1.0, "get vel = %f", vel_.load());
        return vel_.load();
    }
    //penghu 24/7/8
    bool GetSign()             
    {
        LOG_DEBUG_THROTTLE(1.0, "get vel = %f, vel_y = %f, vel_w = %f", vel_.load(), vel_y_.load(), vel_w_.load());
        if(vel_.load() == 0 && vel_y_.load() == 0 && fabs(vel_w_.load()) < 0.005)
        {
            return true;
        }else{
            return false;
        }
    }
	bool ReadMapFromPgm(const std::string &map_name, OccupancyGrid *occupancy_grid);
	bool ReadMapFromXml(const std::string &map_name, OccupancyGrid *occupancy_grid);

    void SaveTrajectoryToTxt(const std::string file_name, const ReflectorMapData &traj);
    bool ReadTrajectoryFromTxt(const std::string &traj_name, ReflectorMapData &traj);
	bool SaveReflectorMapToXml(const std::string &xml_name, const ReflectorMapData &reflector_map,
        const int write_special_part = PartUnknown);
    int ReadNodeInfoFromXml(const std::string &dir, const std::string &xml_name,
                            ReflectorMapData &reflector_map,
        const LandmarkSpecialPart read_special_part = PartUnknown, const std::string &root_name = "");
	void SaveAGVPose(const std::string &pose_name, const Pose3D &agv_pose);
    //ph add seed pose 2022/2/12
    void SaveSeedPose(const std::string &pose_name, const Pose3D &agv_pose);
    //---------------------
    bool ReadInitialPoseFromTxt(const std::string &dir_name, const std::string &txt_name,
                                int &time, Pose3D &pose);
    bool ReadInitialPoseFromTxt(const std::string &dir_name, const std::string &txt_name,
                                Pose3D &init_pose);
    ///xcc 0331
    bool ReadCodePoseMapFromTxt(const std::string &dir_name,
    const std::string &txt_name, CodePoseMap &code_pose_map);

    //penghu 24/5/22
    void SetCore(const double& score)
    {
        LOG_DEBUG_THROTTLE(1.0, "set score = %f", score);
        score_.store(score);
    }
    //penghu 24/5/22
    double GetScore()
    {
        LOG_DEBUG_THROTTLE(1.0, "get score_ = %f", score_.load());
        return score_.load();
    }

    void WriteInitPoseToTxt(const std::string &dir_name, const std::string &pose_name,
                            const Pose3D &pose);
	OccupancyGrid *GetOccupancyGrid(const std::string &map_name, const bool read_pgm = true);
	void SaveMultiPosesToFile(const std::string &pose_name, 
		const std::vector<std::vector<Pose3D>> &multi_poses_container);
	bool ReadPose3DFromFile(const std::string &file_name,
		std::vector<Pose3D> &poses_container);

#ifdef WIN32
    void TraverseFilesInsideDirectory(const std::string &path, std::vector<std::string> &files);
    void TraverseFilesInOneDirectory(const std::string &path, std::vector<std::string> &files);
    void DeleteAllFilesInDirectory(const std::string &dir);
#else
	int TraverseDirectory(const char* path);
	void DeleteAllFilesInDirectory(const std::string &dir);
    int ExtractAllfileNameInDir(const std::string &dir, std::vector<std::string> &file_names);
    int ExtractAllDirNameInDir(const std::string &dir, std::vector<std::string> &dir_names);
    void TraverDir(const std::string &path,std::vector<std::string> &files);
    std::vector<std::string> TraveDir(const char *path, int depth);
#endif

	void SavePosesToFile(const std::string &pose_name, const std::vector<Pose3D> &poses);
	void SaveLaserRangeDataToFile(const std::string &laser_hits_name,
		const std::vector <SlamLaserScanData> &laser_hits);
    bool ReadLaserRangeDataFromFile(const std::string &laser_hits_name,
        std::queue<SlamLaserScanData> &laser_queue);
    void SaveLaserRangeAndItensityDataToFile(const std::string &laser_hits_name,
                                             const std::vector<SlamLaserScanData> &laser_hits);
    bool ReadLaserRangeAndItensityDataFromFile(const std::string &laser_hits_name,
        std::deque<SlamLaserScanData> &data);
	void SaveLaserKeyParams(const SlamLaserScanData &laser_data);
    void ReadLaserKeyParams(SlamLaserScanData *laser_data, std::string &map_name);
	void SaveIMUDataToFile(const std::string &imu_name,
		const std::vector<SlamIMUData> &data);
	bool ReadIMUDataFromFile(const std::string &imu_name,
        std::queue<SlamIMUData> &imu_queue);
    void SaveIMUKeyParams(const SlamIMUData &imu_data);
	void ReadIMUKeyParams(SlamIMUData *imu_data);
    void SaveOdometryDataToFile(const std::string &odometry_name,
                                const std::vector<SlamOdometryPoseData> &data);
    bool ReadOdometryDataFromFile(const std::string &odometry_name,
                                  std::queue<SlamOdometryPoseData> &data_queue);
    void SaveOdometryKeyParams(const SlamOdometryPoseData &data);
    void ReadOdometryKeyParams(SlamOdometryPoseData *data);
	bool DeleteFile(const std::string &file_name)
	{
		const std::string dir = GetDocsPath() + file_name;
		return m_config_oper->DeleteFile(dir);
	}

    bool ReadVersionInfo(SlamCommon::ProgramVersionEnum &pv, std::string &fv);
    bool WriteVersionInfo(const SlamCommon::ProgramVersionEnum &pv = SlamCommon::PROGRAMVERSION_NONE,
                          const std::string &fv = "");
    bool ReadFile(const std::string &file_name, std::ifstream &file_stream, uint64_t &file_size);
    int ReadMappingTableFromXml(const std::string &dir, const std::string &xml_name,
                                std::map<int, std::vector<int>> &mapping_table);
    int ReadMappingTableFromXml(const std::string &dir, const std::string &xml_name,
                                std::map<int, std::vector<NodeInsertToBaseStru>> &mapping_table);
    int SaveErrorList(const std::map<unsigned short, std::string> &error_info_map,
                      const std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
                      const std::string &file_name, const std::string &project_name);
    int GetErrorList(std::map<unsigned short, std::string> &error_info_map,
                      std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
                      std::string &project_name, const std::string &file_name = "");
    void GetXMLRoot(std::map<unsigned short, std::string> &error_info_map,
                    std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
                    std::string &project_name, boost::property_tree::ptree root_tree);
    int ReadConnNodeInfoFromXml(const std::string &dir, const std::string &xml_name,
                                std::map<int, std::map<int, PathPlanNodeInfoStru> > &conn_info);

    void WriteMultiLineStringToTxt(const std::string &dir_name, const std::string &txt_name,
                                   const std::vector<std::string> &strs);
    bool ReadStringFromTxt(const std::string &dir_name, const std::string &txt_name,
                           std::vector<std::string> &strs);

    int ReadObsAreaFromXml(const std::string &dir, const std::string &xml_name,
                           std::map<int, ObsGroupItemStru> &obs_areas);

    bool SetLocalToGlobalVec(const std::vector<cartographer::transform::Rigid2d>& local_to_global){
        std::unique_lock<std::mutex> lock(local_to_global_mutex_, std::defer_lock);
        if(lock.try_lock()){
            local_to_global_vec_ = local_to_global;

            return true;
        }
        return false;
    }
    bool GetLocalToGlobalVec(std::vector<cartographer::transform::Rigid2d>& local_to_global){
        std::unique_lock<std::mutex> lock(local_to_global_mutex_, std::defer_lock);
        if(lock.try_lock()){
            local_to_global = local_to_global_vec_;

            return true;
        }
        return false;
    }

    bool SetSubmap2dProtos(const std::deque<cartographer::mapping::proto::Submap2D>& submap2d_protos){
        std::unique_lock<std::mutex> lock(submap2d_protos_mutex_, std::defer_lock);
        if(lock.try_lock()){
            limits_.clear();
            precomputation_grid_stacks_for_csm_.clear();
            precomputed_probability_grids_.clear();

            submap2d_protos_ = submap2d_protos;
            for(const auto& submap2d_proto: submap2d_protos_){
                precomputed_probability_grids_.emplace_back(
                    absl::make_unique<cartographer::mapping::ProbabilityGrid>(submap2d_proto.grid(), &conversion_tables_));
                SetMapLimitsAndPrecomputationStack(submap2d_proto);
            }
            set_submap2d_protos_finished_ = true;
            return true;
        }
        return false;
    }

    bool GetSubmap2dProtos(std::deque<cartographer::mapping::proto::Submap2D>& submap2d_protos){
        std::unique_lock<std::mutex> lock(submap2d_protos_mutex_, std::defer_lock);
        if(lock.try_lock() && set_submap2d_protos_finished_){
            submap2d_protos = submap2d_protos_;

            return true;
        }
        return false;
    }

    bool SetFastScanMatcher2dLocalOptions(const std::vector<double>& vec){
        std::unique_lock<std::mutex> lock(fast_scan_matcher_2d_local_mutex_, std::defer_lock);
        if(lock.try_lock()){
            fast_scan_matcher_2d_local_vec_ = vec;
            return true;
        }
        
        return false;
    }

    bool GetFastScanMatcher2dLocalOptions(std::vector<double>& vec){
        std::unique_lock<std::mutex> lock(fast_scan_matcher_2d_local_mutex_, std::defer_lock);
        if(lock.try_lock()){
            vec = fast_scan_matcher_2d_local_vec_;
            return true;
        }
        return false;
    }

    bool SetLocalTrajectoryBuilder2dOptions(const std::vector<double>& vec){
        std::unique_lock<std::mutex> lock(local_trajectory_builder_2d_mutex_, std::defer_lock);
        if(lock.try_lock()){
            local_trajectory_builder_2d_vec_ = vec;
            return true;
        }
        return false;
    }

    bool GetLocalTrajectoryBuilder2dOptions(std::vector<double>& vec){
        std::unique_lock<std::mutex> lock(local_trajectory_builder_2d_mutex_, std::defer_lock);
        if(lock.try_lock()){
            vec = local_trajectory_builder_2d_vec_;
            return true;
        }
        return false;
    }

    void SetPureLocalization(const int& match_mode){
        LOG_DEBUG("set match_mode = %d", match_mode);
        match_mode_.store(match_mode);
    }

    int GetPureLocalization(){

        return match_mode_.load();
    }

    void SetMapLimitsAndPrecomputationStack(const cartographer::mapping::proto::Submap2D& submap_proto) {
        cartographer::mapping::ValueConversionTables conversion_tables;
        cartographer::mapping::Submap2D submap_2d(submap_proto, &conversion_tables);

        limits_.emplace_back(submap_2d.grid()->limits());
        precomputation_grid_stacks_for_csm_.emplace_back(
            std::make_shared<cartographer::mapping::scan_matching::PrecomputationStack>(*(submap_2d.grid()), 7));
    }

    bool GetMapLimits(std::vector<cartographer::mapping::MapLimits>& limits){
        std::unique_lock<std::mutex> lock(submap2d_protos_mutex_, std::defer_lock);
        if(lock.try_lock() && set_submap2d_protos_finished_){
            limits = limits_;

            return true;
        }
        return false;
    }

    bool GetPrecomputationStackForCSM(std::vector<std::shared_ptr<cartographer::mapping::scan_matching::PrecomputationStack> >& precomputation_grid_stacks){
        std::unique_lock<std::mutex> lock(submap2d_protos_mutex_, std::defer_lock);
        if(lock.try_lock() && set_submap2d_protos_finished_){
            precomputation_grid_stacks = precomputation_grid_stacks_for_csm_;
            return true;
        }
        return false;
    }

    void SetRefColsPosition(double x, double y) {
        ref_cols_position_x_.store(x);
        ref_cols_position_y_.store(y);
    }

    void GetRefColsPosition(double& x, double& y) {
        x = ref_cols_position_x_.load();
        y = ref_cols_position_y_.load();
    }

    void SetRefColsResponseResults(std::string str) {
        std::unique_lock<std::mutex> lock(ref_cols_response_results_mutex_, std::defer_lock);
        if(lock.try_lock()){
            ref_cols_response_results_ = str;
        }
    }

    std::string GetRefColsResponseResults() {
        std::unique_lock<std::mutex> lock(ref_cols_response_results_mutex_, std::defer_lock);
        if(lock.try_lock()){
            return ref_cols_response_results_;
        }
        return std::string("");
    }
    
    bool GetPrecomputedProbabilityGrids(
        std::vector<std::unique_ptr<cartographer::mapping::ProbabilityGrid>> &precomputed_probability_grids)
    {
        std::unique_lock<std::mutex> lock(submap2d_protos_mutex_, std::defer_lock);
        if (lock.try_lock() && set_submap2d_protos_finished_) {
            precomputed_probability_grids = std::move(precomputed_probability_grids_);
            return true;
        }
        return false;
    }  // 避免扫描匹配的时候重复构造对象耗时

private:
	void WritePgm(const std::string &map_name, const std::string &pgm_name,
        const OccupancyGrid &occupancy_grid);
    void WritePgmWithFile(const std::string &pgm_name, const OccupancyGrid &occupancy_grid);
	void WriteXml(const std::string &map_name, const std::string& xml_name,
		const OccupancyGrid &occupancy_grid);
    int ReadNodePose(Pose3D &pose);
    int ReadNodeInfo(PathNode &node);

private:
	std::string m_cur_path;
	CConfigFileOperator *m_config_oper;
	DefMapDataMap m_map_list;
    bool m_negate_map;
    std::vector<cartographer::transform::Rigid2d> local_to_global_vec_;
    std::deque<cartographer::mapping::proto::Submap2D> submap2d_protos_;

    std::vector<cartographer::mapping::MapLimits> limits_;
    std::vector<std::shared_ptr<cartographer::mapping::scan_matching::PrecomputationStack> > precomputation_grid_stacks_for_csm_;
    std::vector<std::unique_ptr<cartographer::mapping::ProbabilityGrid>> precomputed_probability_grids_; //9/16
    cartographer::mapping::ValueConversionTables  conversion_tables_;
    std::vector<double> fast_scan_matcher_2d_local_vec_;
    std::vector<double> local_trajectory_builder_2d_vec_;
    std::mutex local_to_global_mutex_;
    std::mutex submap2d_protos_mutex_;
    std::mutex fast_scan_matcher_2d_local_mutex_;
    std::mutex local_trajectory_builder_2d_mutex_;
    std::mutex ref_cols_response_results_mutex_;

    //penghu 24/5/19
    std::atomic<double> vel_;
    std::atomic<double> vel_y_;
    std::atomic<double> vel_w_;
    //penghu 24/5/22
    std::atomic<double> score_;
    //penghu 24/6/26
    std::atomic<bool> autorunState_;
    std::atomic<int> match_mode_;
    std::atomic<bool> set_submap2d_protos_finished_;

    std::atomic<double> ref_cols_position_x_;
    std::atomic<double> ref_cols_position_y_;
    std::string ref_cols_response_results_;
};

}// namespace SlamCommon

#endif // !FILE_INTERFACE_H_
