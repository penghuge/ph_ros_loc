#include "log.h"
#include <stdlib.h>

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#endif

namespace Logging
{
CLogging::CLogging()
{
//    std::cout<<"constructor called!"<<std::endl;
}

void CLogging::DeleteSpaceInString(std::string &str)
{
	std::vector<std::string> space_type;
	space_type.push_back(" ");
	space_type.push_back("\t");
	int index = 0;
	bool find = false;
	int i = 100000;
	size_t str_len = 0;
	while (--i)
	{
		find = false;
		for (size_t i = 0; i < space_type.size(); ++i)
		{
			index = str.find(space_type[i]);
			if (index >= 0)
			{
				str_len = str.length();
				str = str.substr(str.find_first_not_of(space_type[i]),
					str.find_last_not_of(space_type[i]) + 1);
				find = true;
				if (str_len == str.length())
				{
					str = str.substr(0, str.find_first_of(space_type[i]));
				}
			}
		}
		if (!find)
		{
			return;
		}
	}
}

void CLogging::SetConfigParamToLogging(const std::map<std::string, std::string> &param_map)
{
	std::stringstream ss;
	m_log_file_timeout = 0;
	for (std::map<std::string, std::string>::const_iterator param = param_map.begin(); param != param_map.end(); ++param)
	{
		ss.clear();
		ss.str(param->second);
		if ("log_name" == param->first)
		{
			ss >> m_log_name;
		}
        //版本号从程序里设置，以确保程序版本不会乱
//		else if ("software_version" == param->first)
//		{
//			ss >> m_software_version;
//		}
		else if ("print_to_file" == param->first)
		{
			ss >> std::boolalpha >> m_enable_logging_to_file;
		}
		else if ("print_to_screen" == param->first)
		{
			ss >> std::boolalpha >> m_enable_logging_to_screen;
		}
		else if ("print_log_param" == param->first)
		{
			ss >> std::boolalpha >> m_print_log_param;
		}
		else if ("enable_debug_log" == param->first)
		{
			ss >> std::boolalpha >> m_enable_debug_logging;
		}
		else if ("log_file_timeout_year" == param->first)
		{
			int year = 0;
			ss >> year;
			m_log_file_timeout += (int)(year * UNIT_TF_YEAR_TO_SECOND);
		}
		else if ("log_file_timeout_month" == param->first)
		{
			int month = 0;
			ss >> month;
			m_log_file_timeout += (int)(month * UNIT_TF_MONTH_TO_SECOND);
		}
		else if ("log_file_timeout_day" == param->first)
		{
			int day = 0;
			ss >> day;
			m_log_file_timeout += (int)(day * UNIT_TF_DAY_TO_SECOND);
		}
		else if ("log_file_timeout_hour" == param->first)
		{
			int hour = 0;
			ss >> hour;
			m_log_file_timeout += (int)(hour * UNIT_TF_HOUR_TO_SECOND);
		}
		else if ("log_file_timeout_minute" == param->first)
		{
			int minute = 0;
			ss >> minute;
			m_log_file_timeout += (int)(minute * UNIT_TF_MINUTE_TO_SECOND);
		}
		else if ("log_file_timeout_second" == param->first)
		{
			int second = 0;
			ss >> second;
			m_log_file_timeout += second;
		}
		else if ("print_to_screen_level" == param->first)
		{
			int level = 0;
			ss >> level;
			if (level < (int)LOGLEVEL_UNKNOWN || level >(int)LOGLEVEL_DISABLE)
			{
				level = (int)LOGLEVEL_INFO;
			}
			m_output_screen_level = (LogLevel)level;
            SetLogLevel(level);//lyy
		}
		else if ("max_float_precision" == param->first)
		{
			ss >> m_max_float_precision;
		}
		else if ("flush_period" == param->first)
		{
			if (nullptr != m_log_writer)
			{
				unsigned long period_ms = 0;
				ss >> period_ms;
				m_log_writer->SetFlushPeriod(period_ms);
			}
		}
		else if ("log_file_max_size" == param->first)
		{
			if (nullptr != m_log_writer)
			{
				uint64_t size = 0;
				ss >> size;
				m_log_writer->SetLogFileMaxSize(size);
			}
		}
        else if ("record_data_to_file" == param->first)
        {
            ss >> std::boolalpha >> m_record_data_to_file;
        }
        else if ("record_file_flush_period" == param->first)
        {
            if (nullptr != m_log_writer)
            {
                unsigned long period_ms = 0;
                ss >> period_ms;
                m_log_writer->SetRecordFileFlushPeriod(period_ms);
            }
        }
	}
}

/*********************************************************************
Function: Parse log configuration file
*********************************************************************/
int CLogging::ParseConfigFile(const std::string &cfg_path, std::map<std::string, std::string> &param_map)
{
	std::string cfg_name = cfg_path;
	std::string str_tempt("");
	str_tempt += cfg_name.back();
	if (DIR_FLAG != str_tempt)
	{
		cfg_name += DIR_FLAG;
	}
	cfg_name += "log_param.config";
	std::ifstream infile;

	infile.open(cfg_name.c_str());

	if (!infile.is_open())
	{
		return 1;
	}
	std::string map_first(""), map_second("");
	int index = 0;
	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		if ("" == str_tempt)
		{
			continue;
		}
		//drop the front space and comment
		str_tempt = str_tempt.substr(str_tempt.find_first_not_of(" "), str_tempt.find_first_of("#"));
		if ("" == str_tempt)
		{
			continue;
		}
		//find sperate flag ":"
		index = str_tempt.find_first_of(":");
		if (index < 0)
		{
			continue;
		}
		map_first = str_tempt.substr(str_tempt.find_first_not_of(" "), index);
		DeleteSpaceInString(map_first);
		map_second = str_tempt.substr(index + 1, str_tempt.length());
		DeleteSpaceInString(map_second);
		param_map[map_first] = map_second;
	}
	infile.close();

	SetConfigParamToLogging(param_map);
	return 0;
}

void CLogging::InitLogging(const char* argv0, const std::string &cur_dir,
                 const std::string &log_cfg_dir, const std::string &log_file_dir)
{
	m_log_writer = new CLogWriter();

    m_root_dir = cur_dir;
	std::string dir(log_file_dir);
	std::string last_str("");
	last_str += dir.back();
	if (DIR_FLAG != last_str)
	{
		dir += DIR_FLAG;
	}
    m_log_dir = ("" == dir) ? SINGLE_DOT_FLAG : dir;

	//read log configuration file params
	dir = log_cfg_dir;
	last_str += dir.back();
	if (DIR_FLAG != last_str)
	{
		dir += DIR_FLAG;
	}
	std::map<std::string, std::string> param_map;
	int ret = ParseConfigFile(dir, param_map);
	if (0 != ret)
	{
		std::cout << "Error: parse config file failed(error: " << ret << ")! Using default value!" << std::endl;
	}

    if (!m_enable_logging_to_file)
    {
        return;
    }
    std::string day = GetFormatDate();
    std::string now = GetFormatTime();
    std::string str_tmpt = "";
    for (size_t i = 0; i < day.length(); ++i)
    {
        if ('-' != day[i])
        {
            str_tmpt += day[i];
        }
    }
    for (size_t i = 0; i < now.length(); ++i)
    {
        if (':' != now[i])
        {
            str_tmpt += now[i];
        }
    }
    str_tmpt = m_log_name + UNDERLINE_FLAG + str_tmpt;
    m_log_writer->Init(str_tmpt, m_log_dir);
    m_log_writer->PushStreamIntoLoggingBuffer(">>> Log is created at " + day + " " + now + RETURN_FLAG);
    std::string proj_name = std::string(argv0);
    int index = proj_name.find_last_of(DIR_FLAG);
    if(index >= 0)
    {
        proj_name = proj_name.substr(index + 1, proj_name.length());
    }
    m_log_writer->PushStreamIntoLoggingBuffer(">>> Log for project: " + proj_name + " V" + m_software_version + RETURN_FLAG);

	if (m_print_log_param)
	{
		LOG_INFO("==================Log configuration parameters==================#h");
		for (std::map<std::string, std::string>::const_iterator param = param_map.begin(); param != param_map.end(); ++param)
		{
			LOG_INFO("%s: %s#h", param->first.c_str(), param->second.c_str());
		}
		LOG_INFO("================================================================#h");
	}
	if (m_log_file_timeout > 0)
	{
		DeleteTimeoutLogFiles(m_log_dir);
	}
}

/*********************************************************************
Function: release logging resource
*********************************************************************/
void CLogging::ShutDownLogging()
{
	DELETE_PTR(m_log_writer);
}

#ifdef WIN32
void CLogging::TcharToChar(const TCHAR * tchar, char * _char)
{
	int iLength;
	//get char length
	iLength = WideCharToMultiByte(CP_ACP, 0, tchar, -1, NULL, 0, NULL, NULL);
	//translate tchar to _char
	WideCharToMultiByte(CP_ACP, 0, tchar, -1, _char, iLength, NULL, NULL);
}

bool CLogging::GetCurrentPath(std::string &cur_path)
{
	TCHAR cur_dir_ori[PATH_MAX] = { 0 };
	if (!GetCurrentDirectory(PATH_MAX, cur_dir_ori))
	{
        LOG_ERROR("Read current path failed!");
		return false;
	}

	char cur_dir[PATH_MAX] = { 0 };
	TcharToChar(cur_dir_ori, cur_dir);

	const char *ptr = std::strrchr(cur_dir, '\\');
	int len = std::strlen(cur_dir) - std::strlen(ptr);
	//get parent directory
	std::string debug_path(cur_dir);
	cur_path = debug_path.substr(0, len);

	cur_path.append(std::string(DIR_FLAG));
	return true;
}

time_t CLogging::GetFileModifyTime(const std::string &file_name)
{
	std::string file(m_log_dir + file_name);
	FILE *fp = fopen(file.c_str(), "r");
	int fd = _fileno(fp);
	struct stat buf;
	fstat(fd, &buf);
	fclose(fp);
//  LOG_INFO("Read file: %s, modify time: %ds!", file.c_str(), buf.st_mtime);
	return buf.st_mtime;
}

void CLogging::DeleteTimeoutLogFiles(const std::string &dir)
{
	long file_t = 0;
	struct _finddata_t file_info;
    std::string path_name;

	if ((file_t = _findfirst(path_name.assign(dir).append("\\*").c_str(), &file_info)) == -1)
	{
		return;
	}
    std::string name_t("");
	do
	{
		if (file_info.attrib & _A_SUBDIR)
		{
            name_t = std::string(file_info.name);
            if (name_t != DOUBLE_DOT_FLAG && name_t != SINGLE_DOT_FLAG)
			{
				DeleteTimeoutLogFiles(dir + "\\" + name_t);
			}
		}
		else
		{
            name_t = std::string(file_info.name);
			double second = GetFileModifyToNowDiffTime(name_t);
//          LOG_INFO("Modify diff time %fs, timeout = %lds", second, m_log_file_timeout);
			if (second >= m_log_file_timeout)
			{
				int day = (int)(second / UNIT_TF_DAY_TO_SECOND);
				second -= day * UNIT_TF_DAY_TO_SECOND;
				int hour = (int)(second / UNIT_TF_HOUR_TO_SECOND);
				second -= hour * UNIT_TF_HOUR_TO_SECOND;
				int minute = (int)(second / UNIT_TF_MINUTE_TO_SECOND);
				second -= minute * UNIT_TF_MINUTE_TO_SECOND;
                LOG_INFO("The log file(%s) is %d day %d hour %d minute %d second ago, remove it!",
                         name_t.c_str(), day, hour, minute, (int)second);
				if (0 == DeleteFileA((dir + name_t).c_str()))
				{
                    LOG_ERROR("Delete file %s failed!", (dir + name_t).c_str());
				}
			}
		}
	} while (_findnext(file_t, &file_info) == 0);
	_findclose(file_t);
}

#else
bool CLogging::GetCurrentPath(std::string &cur_path)
{
	char cur_dir[PATH_MAX] = { 0 };
	if (!getcwd(cur_dir, PATH_MAX))
	{
        LOG_ERROR("Read current path failed!");
		return false;
	}

	//get the last directory's '/'
	const char *ptr = std::strrchr(cur_dir, '/');
	int len = std::strlen(cur_dir) - std::strlen(ptr);
	//get parent directory
	std::string debug_path(cur_dir);
	cur_path = debug_path.substr(0, len);

    cur_path.append(std::string(DIR_FLAG));
	return true;
}

time_t CLogging::GetFileModifyTime(const std::string &file_name)
{
    std::string file(m_log_dir + file_name);
    FILE *fp = fopen(file.c_str(), "r");
    int fd = fileno(fp);
    struct stat buf;
    fstat(fd, &buf);
    fclose(fp);
//    LOG_INFO("Read file: %s, modify time: %ds!", file.c_str(), NumToString(buf.st_mtime));
    return buf.st_mtime;
}

void CLogging::DeleteTimeoutLogFiles(const std::string &dir)
{
    std::string cur_dir = dir;
    DIR *dp = nullptr;
    struct dirent *entry = nullptr;
    struct stat statbuf;
    //open directory
    if ((dp = opendir(cur_dir.c_str())) == NULL)
    {
        LOG_ERROR("Cannot open directory: %s", cur_dir);
        return;
    }
    //change into the direcotry
    if (-1 == chdir(cur_dir.c_str()))
    {
        LOG_ERROR("Change directory error: not a dir or access!");
        return;
    }
    //traverse this directory
    while ((entry = readdir(dp)) != NULL)
    {
        //read the file type in this directory
        lstat(entry->d_name, &statbuf);

        //ignore directory flat
        if (0 == strcmp(entry->d_name, SINGLE_DOT_FLAG) || 0 == strcmp(entry->d_name, DOUBLE_DOT_FLAG))
        {
//            LOG_INFO("Read directory: %s, continue!", entry->d_name);
            continue;
        }

        //if it is a directory, go into it
        if (S_ISDIR(statbuf.st_mode))
        {
//            LOG_INFO("Read directory %s, enter it!", entry->d_name);
            //change directory
            if (-1 == chdir(entry->d_name))
            {
                LOG_ERROR("Change directory failed!");
                return;
            }
            //traverse into the deeper directory
            DeleteTimeoutLogFiles(SINGLE_DOT_FLAG);
            //return to the last directory
            if (-1 == chdir(DOUBLE_DOT_FLAG))
            {
                LOG_ERROR("Return to the last directory failed!");
                return;
            }
        }

        //if it is a file, delete it
        if (S_ISREG(statbuf.st_mode))
        {
            double second = GetFileModifyToNowDiffTime(entry->d_name);
//            LOG_INFO("Modify diff time %fs, timeout = %lds", second, m_log_file_timeout);
            if(second >= m_log_file_timeout)
            {
                int day = (int)(second / UNIT_TF_DAY_TO_SECOND);
                second -= day * UNIT_TF_DAY_TO_SECOND;
                int hour = (int)(second / UNIT_TF_HOUR_TO_SECOND);
                second -= hour * UNIT_TF_HOUR_TO_SECOND;
                int minute = (int)(second / UNIT_TF_MINUTE_TO_SECOND);
                second -= minute * UNIT_TF_MINUTE_TO_SECOND;
                LOG_INFO("The log file(%s) is %d day %d hour %d minute %d second ago, remove it!",
                         entry->d_name, day, hour, minute, (int)second);
                if(remove(entry->d_name) < 0)
                {
                    LOG_WARNING("Delete file %s failed, error: %s", entry->d_name, strerror(errno));
                }
            }
        }
    }
    //change into the origin direcotry
    if (-1 == chdir(m_root_dir.c_str()))
    {
        LOG_ERROR("Change directory error: not a dir or access!");
        return;
    }
    closedir(dp);
    dp = nullptr;
    entry = nullptr;
}

#endif

double CLogging::GetFileModifyToNowDiffTime(const std::string &file_name)
{
    return difftime(GetNowTime(), GetFileModifyTime(file_name));
}
uint64_t CLogging::GetFileSize(const std::string &file_name)
{
    return m_log_writer->GetFileSize(file_name);
}

/*********************************************************************
Function: extract input string to log string
*********************************************************************/
std::stringstream CLogging::ExtractLogString(const char *pstr, va_list &ap)
{
	std::stringstream ss;
	char  pchar1;//find %  
	char  pchar2;//get the kind of the data which Behand of %
	const char* pSource = pstr;
	char* pTemp = NULL;//save the getdata  

	while ((pchar1 = *pSource++))
	{
		if (pchar1 == '%' && (pchar2 = *pSource) != '\0')
		{
			pSource++;
			switch (pchar2)
			{
			case 's':
			{
				pTemp = va_arg(ap, char*);
				ss << pTemp;
			}
			break;
			case 'd':
			{
				int nTemp = va_arg(ap, int);
				ss << nTemp;
			}
			break;
			case 'l':
			{
				std::string buf("");
				buf += pchar1;
				buf += pchar2;
				pchar1 = *pSource++;
				switch (pchar1)
				{
				case 'd':
				{
					int32_t nTemp = va_arg(ap, int32_t);
					ss << nTemp;
				}
				break;
				case 'u':
				{
					uint32_t nTemp = va_arg(ap, uint32_t);
					ss << nTemp;
				}
				break;
				case 'l':
				{
					buf += pchar1;
					pchar1 = *pSource++;
					switch (pchar1)
					{
					case 'd':
					{
						int64_t nTemp = va_arg(ap, int64_t);
						ss << nTemp;
					}
					break;
					case 'u':
					{
						uint64_t nTemp = va_arg(ap, uint64_t);
						ss << nTemp;
					}
					break;
					default:
						ss << buf << pchar1;
					break;
					}
				}
				break;
				default:
					ss << buf << pchar1;
				break;
				}
			}
			break;
			case 'u':
			{
				unsigned int nTemp = va_arg(ap, unsigned int);
				ss << nTemp;
			}
			break;
			case 'f':
			{
				double d = va_arg(ap, double);
				std::ostringstream oss;
				ss << std::fixed << std::setprecision(m_max_float_precision) << d;
			}
			break;
			case 'x':
			{
				int nTemp = va_arg(ap, int);
				char str[100];
				sprintf(str, "0x%x", nTemp);
				ss << str;
			}
			break;
			case 'X':
			{
				int nTemp = va_arg(ap, int);
				char str[100];
				sprintf(str, "0x%X", nTemp);
				ss << str;
			}
			break;
			case 'c':
			{
				char cTemp = va_arg(ap, int);//for memory alignment
				ss << cTemp;
			}
			break;
			default:
			{
				ss << pchar1;
				ss << pchar2;
			}
			break;
			}
		}
		else
		{
			ss << pchar1;
		}
	}
	return ss;
}

std::string CLogging::FormatString(const char *fmt, va_list &ap)
{
    std::string msg("");

    size_t size = 10240;
    static char *buffer = new char[size];

    if (NULL == buffer)
    {
        return msg;
    }
    int n = vsnprintf(buffer, size, fmt, ap);

    if (n > 0 && (unsigned)n < size)
    {
        msg.assign(buffer, n);
    }
    else if((unsigned)n > size)
    {
        msg.assign(buffer,size);
    }
    else
    {
        msg = "vsnprintf error!";
    }

    return msg;
}

/*********************************************************************
Function: Logging all type log
*********************************************************************/
void CLogging::WriteLogInterface(LogLevel level, const char *file,
                                 const long int line, const char *pstr, ...)
{
    //解析日志
	va_list ap;
	va_start(ap, pstr);
#if 1
    if(log_level <= level) {
        //逐字符解析，打印方法自己定义
        std::stringstream ss = ExtractLogString(pstr, ap);
        //日志输出
        WriteLogInterface(level, file, line, ss.str());
        ss.clear();
        ss.str("");
      }
#else
    //调用系统函数直接解析，打印方法与printf完全一样
    WriteLogInterface(level, file, line, FormatString(pstr, ap));
#endif
    va_end(ap);
}

void CLogging::WriteLogInterface(bool cond, LogLevel level, const char *file,
                           const long int line, const char *pstr, ...)
{
    if(!cond) return;
    va_list ap;
    va_start(ap, pstr);
    if(level >= log_level) {
        std::stringstream ss = ExtractLogString(pstr, ap);
        WriteLogInterface(level, file, line, ss.str());
        ss.clear();
        ss.str("");
    }
    va_end(ap);
}

void CLogging::WriteLogHZ(double hz, LogLevel level, const char *file,
                           const long int line, const char *pstr, ...)
{
    va_list ap;
    va_start(ap, pstr);
    // printf("level: %d >= log_level: %d\n", level, log_level);
    if(level >= log_level) {
        std::stringstream ss = ExtractLogString(pstr, ap);
        // printf("ss: %s\n", ss.str().c_str());

        std::string str(file);
        str = str + std::to_string(line);
        // printf("file_location_time_map.count(str): %d\n", file_location_time_map.count(str));
        if(file_location_time_map.count(str) == 0) {
            file_location_time_map[str] = SlamCommon::TimeNow();
            WriteLogInterface(level, file, line, ss.str());
        } else {
            SlamCommon::Duration duration = SlamCommon::TimeNow() - file_location_time_map[str];
            // printf("dura: %f, 1/hz: %f\n", SlamCommon::ToSeconds(duration), 1.0 / hz);
            if(SlamCommon::ToSeconds(duration) > (1.0 / hz)) {
                file_location_time_map[str] = SlamCommon::TimeNow();
                WriteLogInterface(level, file, line, ss.str());
            }
        }

        ss.clear();
        ss.str("");
    }
    va_end(ap);
}

void CLogging::WriteLogInterface(bool cond, LogLevel level, const char *file, const long line,
                           const std::string &str) {
    if(cond)  WriteLogInterface(level, file, line, str);
}

void CLogging::WriteLogHZ(double hz, LogLevel level, const char *file, const long line,
                           const std::string &str) {
    std::string loc(file);
    loc = loc + std::to_string(line);

    if(file_location_time_map.count(loc) == 0) {
        file_location_time_map[loc] = SlamCommon::TimeNow();
        WriteLogInterface(level, file, line, str);
    } else {
        SlamCommon::Duration duration = SlamCommon::TimeNow() - file_location_time_map[loc];
        if(SlamCommon::ToSeconds(duration) > (1.0 / hz)) {
            file_location_time_map[loc] = SlamCommon::TimeNow();
            WriteLogInterface(level, file, line, str);
        }
    }
}

/*********************************************************************
Function: Logging std::string type log
*********************************************************************/
void CLogging::WriteLogInterface(LogLevel level, const char *file,
                                 const long int line, const std::string &str)
{
    std::string str_t(str);
    std::string log_str(INFO_TAG);
    switch (level)
    {
    case LOGLEVEL_DEBUG:
        if(!m_enable_debug_logging)
        {
            return;
        }
        log_str = DEBUG_TAG;
        break;
    case LOGLEVEL_INFO:
        log_str = INFO_TAG;
        break;
    case LOGLEVEL_WARNING:
        log_str = WARNING_TAG;
        break;
    case LOGLEVEL_ERROR:
        log_str = ERROR_TAG;
        break;
    case LOGLEVEL_FATAL:
        log_str = FATAL_TAG;
        break;
    default:
        log_str = INFO_TAG;
        break;
    }

    int index = str_t.length() - 2;
    if (index >= 0)
    {
        std::string end_str(str_t.substr(index));
        //check insert return key / header or not
        if (!strcmp(BAN_RETURN_FLAG, end_str.c_str()))
        {
            log_str = str_t.substr(0, index);
        }
        else if (!strcmp(BAN_HEADER_FLAG, end_str.c_str()))
        {
            log_str = str_t.substr(0, index) + RETURN_FLAG;
        }
        else
        {
            const char *f = strrchr(file, DIR_FLAG_CHAR);
            if (NULL == f)
            {
                f = strrchr(file, DIR_INVERSE_FLAG_CHAR);
                if (NULL == f)
                {
                    f = file;
                }
                else
                {
                    f += 1;
                }
            }
            else
            {
                f += 1;
            }
            std::stringstream ss_t;
            ss_t << "I" << std::this_thread::get_id() << " "                //进程ID
                 << GetFormatTime() << "." << GetSysTimeOnlyMicros() << " " //系统时间
                 << f << ": "                                               //文件名
                 << line << "] ";                                           //行号
            log_str = log_str + ss_t.str() + str_t + RETURN_FLAG;
        }
    }
    else
    {
        const char *f = strrchr(file, DIR_FLAG_CHAR);
        if (NULL == f)
        {
            f = file;
        }
        else
        {
            f += 1;
        }
        std::stringstream ss_t;
        ss_t << "I" << std::this_thread::get_id() << " "                //进程ID
             << GetFormatTime() << "." << GetSysTimeOnlyMicros() << " " //系统时间
             << f << ": "                                               //文件名
             << line << "] ";                                           //行号
        log_str = log_str + ss_t.str() + str_t + RETURN_FLAG;
    }
    //print to screen
    if (m_enable_logging_to_screen && level >= m_output_screen_level)
    {
        std::cout << log_str;
    }
    //print to file
    if (!m_enable_logging_to_file)
    {
        return;
    }
    if (nullptr == m_log_writer)
    {
        std::cout << "Logging ERROR: Please execute InitLogging(project_name, log_name, log_directory) first!" << std::endl;
        return;
    }
    m_log_writer->PushStreamIntoLoggingBuffer(log_str);
}

/*********************************************************************
Function: Logging all type log
*********************************************************************/
void CLogging::WriteLogInterface(const char *pstr, ...)
{
    if(!m_record_data_to_file)
    {
        return;
    }
    if (nullptr == m_log_writer)
    {
        std::cout << "Logging ERROR: Please execute InitLogging(project_name, log_name, log_directory) first!" << std::endl;
        return;
    }

    va_list ap;
    va_start(ap, pstr);
#if 1
    //逐字符解析，打印方法自己定义
    std::stringstream ss = ExtractLogString(pstr, ap);
    std::string str_t(ss.str());
    ss.clear();
    ss.str("");
#else
    //调用系统函数直接解析，打印方法与printf完全一样
    std::string str_t = FormatString(pstr, ap);
#endif
    va_end(ap);
    m_log_writer->PushStreamIntoFileBuffer(str_t + RETURN_FLAG);
}

void CLogging::WriteNamedLogInterface(const char *pstr, ...)
{
    if(!m_record_data_to_file)
    {
        return;
    }
    if (nullptr == m_log_writer)
    {
        std::cout << "Logging ERROR: Please execute InitLogging(project_name, log_name, log_directory) first!" << std::endl;
        return;
    }

    va_list ap;
    va_start(ap, pstr);

    char  pchar1;//find %
    char  pchar2;//get the kind of the data which Behand of %
    const char* pSource = pstr;
    char* pTemp = NULL;//save the getdata

    //取出文件名，要求第一个参数是文件名
    std::string file_name("named_file");
    while ((pchar1 = *pSource++))
    {
        if (pchar1 == '%' && (pchar2 = *pSource) != '\0')
        {
            pSource++;
            if ('s' == pchar2)
            {
                pTemp = va_arg(ap, char*);
                std::stringstream ss;
                ss << pTemp;
                file_name = ss.str();
                ss.clear();
                ss.str("");
                break;
            }
        }
    }

#if 1
    //逐字符解析，打印方法自己定义
    std::stringstream ss = ExtractLogString(pSource, ap);
    std::string str_t(ss.str());
    ss.clear();
    ss.str("");
#else
    //调用系统函数直接解析，打印方法与printf完全一样
    std::string str_t = FormatString(pstr, ap);
#endif
    va_end(ap);

    m_log_writer->WriteNamedFile(file_name + ".txt", str_t + RETURN_FLAG);
}

void CLogging::SetLogLevel(int lvl)
{
    log_level = static_cast<LogLevel>(lvl);
}

std::string CLogging::GetFormatTime()
{
    time_t timep = GetNowTime();
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&timep));
    return tmp;
}

std::string CLogging::GetFormatPreciseTime()
{
    std::stringstream ss_t;
    ss_t << GetFormatTime() << "." << GetSysTimeOnlyMicros(); //系统时间
    std::string t(ss_t.str());
    ss_t.clear();
    ss_t.str("");
    return t;
}

std::string CLogging::GetFormatDate()
{
    time_t timep = GetNowTime();
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d", localtime(&timep));
    return tmp;
}

time_t CLogging::GetNowTime()
{
    time_t timep;
    time(&timep);
    return timep;
}

// 获取系统的当前时间，单位微秒(us)
int64_t CLogging::GetSysTimeMicros()
{
#ifdef _WIN32
// 从1601年1月1日0:0:0:000到1970年1月1日0:0:0:000的时间(单位100ns)
#define EPOCHFILETIME   (116444736000000000UL)
    FILETIME ft;
    LARGE_INTEGER li;
    GetSystemTimeAsFileTime(&ft);
    li.LowPart = ft.dwLowDateTime;
    li.HighPart = ft.dwHighDateTime;
    // 从1970年1月1日0:0:0:000到现在的微秒数(UTC时间)
    return (int64_t)((li.QuadPart - EPOCHFILETIME) / 10);
#else
    timeval tv;
    gettimeofday(&tv, 0);
    return (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;
#endif // _WIN32

    return 0;
}

// 获取系统的当前时间中的毫秒和微秒部分，单位微秒(us)
int CLogging::GetSysTimeOnlyMicros()
{
#ifdef _WIN32
// 从1601年1月1日0:0:0:000到1970年1月1日0:0:0:000的时间(单位100ns)
#define EPOCHFILETIME   (116444736000000000UL)
    FILETIME ft;
    LARGE_INTEGER li;
    GetSystemTimeAsFileTime(&ft);
    li.LowPart = ft.dwLowDateTime;
    li.HighPart = ft.dwHighDateTime;
    // 从1970年1月1日0:0:0:000到现在的微秒数(UTC时间)
    return (li.QuadPart - EPOCHFILETIME) /10 % 1000000;
#else
    timeval tv;
    gettimeofday(&tv, 0);
    return (int)tv.tv_usec;
#endif // _WIN32

    return 0;
}

void CLogging::SetFirmwareVersion(const std::string &vers)
{
    m_software_version = vers;
}

void CLogging::GetFirmwareVersion(std::string &vers)
{
    vers = m_software_version;
}

std::string CLogging::SplitSecondTime(const double sec)
{
    std::string time_str("");
    double second(sec);
    int day = (int)(second / UNIT_TF_DAY_TO_SECOND);
    if(day != 0)
    {
        time_str += NumToString(day) + "天";
        second -= day * UNIT_TF_DAY_TO_SECOND;
    }
    int hour = (int)(second / UNIT_TF_HOUR_TO_SECOND);
    if(hour != 0 || !time_str.empty())
    {
        time_str += NumToString(hour) + "时";
        second -= hour * UNIT_TF_HOUR_TO_SECOND;
    }
    int minute = (int)(second / UNIT_TF_MINUTE_TO_SECOND);
    if(minute != 0 || !time_str.empty())
    {
        time_str += NumToString(minute) + "分";
        second -= minute * UNIT_TF_MINUTE_TO_SECOND;
    }
    time_str += NumToString(second, 0) + "秒";
    return time_str;
}

void CLogging::GetProjectPath(std::string &proj_path)
{
    proj_path = m_root_dir;
}

} // namespace Logging
