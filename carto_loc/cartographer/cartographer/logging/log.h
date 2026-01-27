#ifndef LOG_H_
#define LOG_H_

#include "log_writer.h"
#include <map>
#include "cartographer/common/slam_time.h"

namespace Logging
{
//初始化日志接口
#define LOG_INIT(argv, str1, str2, str3)  Logging::CLogging::GetInstance().InitLogging((argv), (str1), (str2), (str3))
//关闭日志接口
#define LOG_SHUTDOWN()          Logging::CLogging::GetInstance().ShutDownLogging()
//设置软件版本接口
#define LOG_SET_FW_VER(str)     Logging::CLogging::GetInstance().SetFirmwareVersion((str))
//获取软件版本接口
#define LOG_GET_FW_VER(str)     Logging::CLogging::GetInstance().GetFirmwareVersion((str))
//获取程序可执行文件路径接口
#define LOG_CUR_PATH(str)       Logging::CLogging::GetInstance().GetCurrentPath((str))
//获取当前软件工程所在目录接口
#define LOG_PROJECT_PATH(str)   Logging::CLogging::GetInstance().GetProjectPath((str))
//获取指定文件的大小接口
#define LOG_FILE_SIZE(str)      Logging::CLogging::GetInstance().GetFileSize((str))
//将时间格式化成“年/月/日/时/分/秒”
#define LOG_FORMAT_TIME(t)      Logging::CLogging::GetInstance().SplitSecondTime((t))
//获取当前系统日期，格式：年-月-日
#define LOG_GET_DATA()          Logging::CLogging::GetInstance().GetFormatDate()
//获取当前系统时间，格式：时:分:秒
#define LOG_GET_TIME()          Logging::CLogging::GetInstance().GetFormatTime()
//获取当前系统时间，格式：时:分:秒.毫秒微秒
#define LOG_GET_PRECESE_TIME()  Logging::CLogging::GetInstance().GetFormatPreciseTime()
//日志打印接口
#define LOG_DEBUG(fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_INFO(fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_WARNING(fmt,...)	Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_ERROR(fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_FATAL(fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)

#define LOG_DEBUG_STREAM(fmt)	Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt))
#define LOG_INFO_STREAM(fmt)	Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt))
#define LOG_WARNING_STREAM(fmt) Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt))
#define LOG_ERROR_STREAM(fmt)	Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt))
#define LOG_FATAL_STREAM(fmt)	Logging::CLogging::GetInstance().WriteLogInterface(Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt))

#define LOG_DEBUG_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_INFO_COND(cond,fmt,...)		    Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_WARNING_COND(cond,fmt,...)	    Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_ERROR_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_FATAL_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)

#define LOG_DEBUG_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_INFO_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_WARNING_THROTTLE(hz,fmt,...)	Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_ERROR_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_FATAL_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt), ##__VA_ARGS__)

#define LOG_DEBUG_STREAM_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt))
#define LOG_INFO_STREAM_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt))
#define LOG_WARNING_STREAM_COND(cond,fmt,...)	Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt))
#define LOG_ERROR_STREAM_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt))
#define LOG_FATAL_STREAM_COND(cond,fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface(cond, Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt))

#define LOG_DEBUG_STREAM_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_DEBUG,   __FILE__, __LINE__, (fmt))
#define LOG_INFO_STREAM_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_INFO,    __FILE__, __LINE__, (fmt))
#define LOG_WARNING_STREAM_THROTTLE(hz,fmt,...)	    Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_WARNING, __FILE__, __LINE__, (fmt))
#define LOG_ERROR_STREAM_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_ERROR,   __FILE__, __LINE__, (fmt))
#define LOG_FATAL_STREAM_THROTTLE(hz,fmt,...)		Logging::CLogging::GetInstance().WriteLogHZ(hz, Logging::LOGLEVEL_FATAL,   __FILE__, __LINE__, (fmt))

#define LOG_SET_LEVEL(lvl)      Logging::CLogging::GetInstance().SetLogLevel(lvl)

//数据记录接口
#define LOG_FILE(fmt,...)		Logging::CLogging::GetInstance().WriteLogInterface((fmt), ##__VA_ARGS__)
//指定文件名日志记录接口
#define LOG_NAMED_FILE(fmt,...)	Logging::CLogging::GetInstance().WriteNamedLogInterface((fmt), ##__VA_ARGS__)
//条件检测接口
#define LOG_CHECK_EQ(val1, val2)	if((val1) != (val2))LOG_ERROR("CHECK == ERROR!");
#define LOG_CHECK_NE(val1, val2)	if((val1) == (val2))LOG_ERROR("CHECK != ERROR!");
#define LOG_CHECK_LE(val1, val2)	if((val1) >  (val2))LOG_ERROR("CHECK <= ERROR!");
#define LOG_CHECK_LT(val1, val2)	if((val1) >= (val2))LOG_ERROR("CHECK <  ERROR!");
#define LOG_CHECK_GE(val1, val2)	if((val1) <  (val2))LOG_ERROR("CHECK >= ERROR!");
#define LOG_CHECK_GT(val1, val2)	if((val1) <= (val2))LOG_ERROR("CHECK >  ERROR!");
#define LOG_CHECK(cond)				if(!(cond))LOG_ERROR("CHECK failed: Condition is false!");
#define LOG_CHECK_NOTNULL(ptr)		if(NULL == (ptr))LOG_ERROR("CHECK failed: Pointer is NULL!");

#define UNIT_TF_YEAR_TO_SECOND      31104000    /* 12 * 30 * 24 * 60 * 60 */
#define UNIT_TF_MONTH_TO_SECOND     2592000     /* 30 * 24 * 60 * 60 */
#define UNIT_TF_DAY_TO_SECOND       86400       /* 24 * 60 * 60 */
#define UNIT_TF_HOUR_TO_SECOND      3600        /* 60 * 60 */
#define UNIT_TF_MINUTE_TO_SECOND    60          /* 60 */

class CLogging
{
public:
    ~CLogging(){}
    CLogging(const CLogging&)=delete;
    CLogging& operator=(const CLogging&)=delete;
    static CLogging& GetInstance()
    {
        //静态单例
        static CLogging instance;
        return instance;
    }

    /*********************************************************************
    Function: 初始化，并启动日志记录
    *********************************************************************/
    void InitLogging(const char* argv0, const std::string &cur_dir,
                     const std::string &log_cfg_dir, const std::string &log_dir);
    /*********************************************************************
    Function: 停止日志记录，释放资源
    *********************************************************************/
    void ShutDownLogging();
    /*********************************************************************
    Function: 日志记录接口
    *********************************************************************/
    void WriteLogInterface(LogLevel level, const char *file,
                           const long int line, const char *pstr, ...);

    void WriteLogInterface(bool cond, LogLevel level, const char *file,
                           const long int line, const char *pstr, ...);

    void WriteLogHZ(double hz, LogLevel level, const char *file,
                           const long int line, const char *pstr, ...);
    /*********************************************************************
    Function: std::string类型日志记录接口
    *********************************************************************/
    void WriteLogInterface(LogLevel level, const char *file, const long line,
                           const std::string &str);

    void WriteLogInterface(bool cond, LogLevel level, const char *file, const long line,
                               const std::string &str);

    void WriteLogHZ(double hz, LogLevel level, const char *file, const long line,
                               const std::string &str);
    /*********************************************************************
    Function: 数据记录接口
    *********************************************************************/
    void WriteLogInterface(const char *pstr, ...);
    /*********************************************************************
    Function: 指定文件名日志记录接口
    *********************************************************************/
    void WriteNamedLogInterface(const char *pstr, ...);
    /*********************************************************************
    Function: 获取程序可执行文件所在目录
    *********************************************************************/
    bool GetCurrentPath(std::string &cur_path);
    /*********************************************************************
    Function: 获取当前软件工程所在目录
    *********************************************************************/
    void GetProjectPath(std::string &proj_path);
    /*********************************************************************
    Function: 设置软件工程的版本
    *********************************************************************/
    void SetFirmwareVersion(const std::string &vers);
    /*********************************************************************
    Function: 获取软件工程的版本
    *********************************************************************/
    void GetFirmwareVersion(std::string &vers);
    /*********************************************************************
    Function: 将“秒”转换成“年/月/日/时/分/秒”格式
    *********************************************************************/
    std::string SplitSecondTime(const double sec);
    /*********************************************************************
    Function: 获取指定文件的大小
    *********************************************************************/
    uint64_t GetFileSize(const std::string &file_name);
    /*********************************************************************
    Function: 设置日志等级，低于设置的等级的日志不记录
    *********************************************************************/
    void SetLogLevel(int log_level);

    std::string GetFormatDate();
    std::string GetFormatTime();
    std::string GetFormatPreciseTime();
private:
    CLogging();
    void DeleteSpaceInString(std::string &str);
    void SetConfigParamToLogging(const std::map<std::string, std::string> &param_map);
    int ParseConfigFile(const std::string &cfg_path, std::map<std::string, std::string> &param_map);
#ifdef WIN32
    void TcharToChar(const TCHAR * tchar, char * _char);
#endif
    time_t GetFileModifyTime(const std::string &file_name);
    double GetFileModifyToNowDiffTime(const std::string &file_name);
    void DeleteTimeoutLogFiles(const std::string &dir);
    time_t GetNowTime();
    int64_t GetSysTimeMicros();
    int GetSysTimeOnlyMicros();

    std::stringstream ExtractLogString(const char *pstr, va_list &ap);
    std::string FormatString(const char *fmt, va_list &ap);

    std::map<std::string,SlamCommon::Time> file_location_time_map;

private:
    //switch
    bool m_enable_logging_to_file = true;
    bool m_enable_logging_to_screen = true;
    bool m_print_log_param = false;
    bool m_enable_debug_logging = false;
    bool m_record_data_to_file = false;

    CLogWriter *m_log_writer = nullptr;
    LogLevel m_output_screen_level = LOGLEVEL_INFO;
    LogLevel log_level = LOGLEVEL_ERROR;
    int m_max_float_precision = 3;
    std::string m_log_dir = "";
    long m_log_file_timeout = 0;
    std::string m_root_dir = "";
    std::string m_software_version = "1.0";
    std::string m_log_name = "LOG";
};

} // namespace Logging
#endif // !LOG_H_
