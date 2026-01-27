#ifndef CARTO_MAPPING_DATA_H_
#define CARTO_MAPPING_DATA_H_

#include "cartographer/common/robot_description.h"

namespace CartoMapppingBridge {

struct SlamStatusResponseStru
{
    enum StatusCode
    {
        OK=0,
        CANCELLED=1,
        UNKNOWN=2,
        INVALID_ARGUMENT=3,
        DEADLINE_EXCEEDED=4,
        NOT_FOUND=5,
        ALREADY_EXISTS=6,
        PERMISSION_DENIED=7,
        RESOURCE_EXHAUSTED=8,
        FAILED_PRECONDITION=9,
        ABORTED=10,
        OUT_OF_RANGE=11,
        UNIMPLEMENTED=12,
        INTERNAL=13,
        UNAVAILABLE=14,
        DATA_LOSS=15,
    };

    StatusCode code = OK;
    std::string message = "";
};

struct SlamMarkerStru
{
    enum ObjType
    {
        ARROW=0,
        CUBE=1,
        SPHERE=2,
        CYLINDER=3,
        LINE_STRIP=4,
        LINE_LIST=5,
        CUBE_LIST=6,
        SPHERE_LIST=7,
        POINTS=8,
        TEXT_VIEW_FACING=9,
        MESH_RESOURCE=10,
        TRIANGLE_LIST=11,
    };

    enum ActionType
    {
        ADD=0,
        MODIFY=0,
        DELETE=2,
        DELETEALL=3,
    };


    SlamCommon::Header header;
    std::string ns = "";            //命名空间名
    int id = 0;                     //对象ID，在后期操作和删除对象时配合名称空间一起使用
    ObjType type = ARROW;           //对象的类型
    ActionType action = ADD;        //0：添加/修改一个对象；1(弃用)；2：删除一个对象；3：删除所有对象
    SlamCommon::Pose6DVecQua pose;  //对象位姿
    Eigen::Vector3d scale = Eigen::Vector3d::Zero();//对象的比例，1,1,1表示默认(通常为1平方米)
    SlamCommon::ColorRGBA color;     //颜色[0.0 - 1.0]
    SlamCommon::Duration lifetime;  //对象在被自动删除之前应该持续多长时间。0意味着永远
    bool frame_locked = false;      //如果这个标记的坐标系固定，也就是说，每一步都要重新转换到它的坐标系中

    //仅当指定的类型有某些用途时才使用(例如：点，线段 …)
    std::vector<Eigen::Vector3d> points;
    //仅当指定的类型有某些用途时才使用(例如：点，线段 …)
    //颜色的数量必须是0或者等于点的数量
    //注：alpha现在还未启用
    std::vector<SlamCommon::ColorRGBA> colors;

    //注:仅用于文本标记
    std::string text = "";

    //注:仅用于MESH_RESOURCE（网格资源）标记
    std::string mesh_resource = "";
    bool mesh_use_embedded_materials = false;
};

struct SlamMarkerArrayStru
{
    std::vector<SlamMarkerStru> markers;
};

struct SlamSubmapEntryStru
{
    int trajectory_id = 0;
    int submap_index = 0;
    int submap_version = 0;
    SlamCommon::Pose6DVecQua pose;
    bool is_frozen = false;
};

struct SlamSubmapListStru
{
    SlamCommon::Header header;
    std::vector<SlamSubmapEntryStru> submap;
};

struct SlamSubmapTextureStru
{
    std::vector<char> cells;
    int width = 0;
    int height = 0;
    double resolution = 0.;
    SlamCommon::Pose6DVecQua slice_pose;
};

struct SlamSubmapQueryRequestStru
{
    int trajectory_id = 0;
    int submap_index = 0;
};

struct SlamSubmapQueryResponseStru
{
    SlamStatusResponseStru status;
    int submap_version = 0;
    std::vector<SlamSubmapTextureStru> textures;
};

struct SlamTrajectoryQueryRequestStru
{
    int trajectory_id = 0;
};

struct SlamTrajectoryQueryResponseStru
{
    SlamStatusResponseStru status;
    std::vector<SlamCommon::Pose6DVecQuaStamped> trajectory;
};

struct SlamMetricLabelStru
{
    std::string key = "";
    std::string value = "";
};

struct SlamHistogramBucketStru
{
    double bucket_boundary = 0.;
    double count = 0.;
};

struct SlamMetricStru
{
    enum MetricTypeEnum
    {
        TYPE_COUNTER,
        TYPE_GAUGE,
        TYPE_HISTOGRAM
    };

    MetricTypeEnum type = TYPE_COUNTER;
    std::vector<SlamMetricLabelStru> labels;

    //TYPE_COUNTER or TYPE_GAUGE
    double value = 0.;

    // TYPE_HISTOGRAM
    std::vector<SlamHistogramBucketStru> counts_by_bucket;
};

struct SlamMetricFamilyStru
{
    std::string name = "";
    std::string description = "";
    std::vector<SlamMetricStru> metrics;
};

struct SlamReadMetricsRequestStru
{
};

struct SlamReadMetricsResponseStru
{
    SlamStatusResponseStru status;
    std::vector<SlamMetricFamilyStru> metric_families;
    SlamCommon::Time timestamp;
};

struct SlamStartTrajectoryRequestStru
{
    std::string configuration_directory = "";
    std::string configuration_basename = "";
    bool use_initial_pose = false;
    SlamCommon::Pose6DVecQua initial_pose;
    SlamCommon::Time pose_timestamp;
    int relative_to_trajectory_id = 0;
};

struct SlamStartTrajectoryResponseStru
{
    SlamStatusResponseStru status;
    int trajectory_id = 0;
};

struct SlamTrajectoryStatesStru
{
    enum StateEnum
    {
        ACTIVE,
        FINISHED,
        FROZEN,
        DELETED
    };

    SlamCommon::Header header;
    std::vector<int> trajectory_id;
    std::vector<StateEnum> trajectory_state;
};

struct SlamGetTrajectoryStatesRequestStru
{
};

struct SlamGetTrajectoryStatesResponseStru
{
    SlamStatusResponseStru status;
    SlamTrajectoryStatesStru trajectory_states;
};

struct SlamFinishTrajectoryRequestStru
{
    int trajectory_id = 0;
};

struct SlamFinishTrajectoryResponseStru
{
    SlamStatusResponseStru status;
};

struct SlamWriteStateRequestStru
{
    std::string filename = "";
    bool include_unfinished_submaps = true;
};

struct SlamWriteStateResponseStru
{
    SlamStatusResponseStru status;
};

} // namespace CartoMapppingBridge

#endif // !CARTO_MAPPING_DATA_H_
