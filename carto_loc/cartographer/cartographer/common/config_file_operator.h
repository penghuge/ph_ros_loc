#ifndef CONFIG_FILE_OPERATOR_H_
#define CONFIG_FILE_OPERATOR_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>  
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
//#include <boost/filesystem.hpp>
#include "format_transform.h"
#include "const_value.h"
#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <stdint.h>
#include <string>
#include "cartographer/logging/log.h"

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

namespace SlamCommon
{

struct ConfigParam
{
    std::string node_name = "";
    std::string value = "";
    std::string comment = "";
};

class CConfigFileOperator
{
public:
	CConfigFileOperator(const std::string &cur_path);
	~CConfigFileOperator();

    std::string GetCurrentPath()
    {
        return m_cur_path;
    }
    /**********************************************************************************
    function: 设置节点所在目录
        参数：node_dir：所在完整目录
    **********************************************************************************/
	void SetNodeDirectory(const std::string &node_dir);
    /**********************************************************************************
    function: 设置节点队列
        参数：root_queue：节点队列
    **********************************************************************************/
    void SetRootQueue(const std::deque<std::string> &root_queue)
    {
        m_root_queue.clear();
        m_root_queue = root_queue;
    }

    /**********************************************************************************
    function: 文件操作前的初始化，只能执行1，2两层节点的初始化操作
        注意：（1）如果没设置过节点目录，需要调用SetNodeDirectory()设置目录
             （2）执行过初始化时，在操作完文件的，最好执行反初始化
        参数：file_name：文件名，仅文件名，不需要包括路径
            root_name：根节点名
    **********************************************************************************/
	bool InitConfigFileOperator(std::string file_name, std::string root_name = "");
    /**********************************************************************************
    function: 文件操作前的初始化，可执行任意多层节点初始化操作
        注意：（1）如果没设置过节点目录，需要调用SetNodeDirectory()设置目录
             （2）执行过初始化时，在操作完文件的，最好执行反初始化
        参数：file_name：文件名，仅文件名，不需要包括路径
            root_queue：各层级节点名
    **********************************************************************************/
    bool InitConfigFileOperator(std::string file_name, std::deque<std::string> root_queue);
    /**********************************************************************************
    function: 写文件前的初始化
    **********************************************************************************/
    void InitConfigFileOperatorForWriting()
    {
        m_root_tree->clear();
        m_is_initialized = true;
    }

    /**********************************************************************************
    function: 反初始化，用于执行过初始化时，文件操作完后执行
    **********************************************************************************/
    void UninitConfigFileOperator()
    {
        m_root_queue.clear();
        m_root_tree->clear();
        m_is_initialized = false;
    }

    /**********************************************************************************
    function: 获取xml根节点树
    **********************************************************************************/
    boost::property_tree::ptree *GetRootTree()
    {
        return m_root_tree;
    }

    /**********************************************************************************
    function: 获取文件中所有节点的信息
        参数：file_name：文件名
            config_params：保存所有节点信息
    **********************************************************************************/
    bool GetChildrenAndValus(const std::string &file_name, std::vector<ConfigParam> &config_params);

    /**********************************************************************************
    function: 获取指定层级下，指定叶子节点名的数量
        参数：root_queue：各层级节点名
            child_name：期望计算数量的节点名
            fuzzy_search：对查找的节点名是否进行模糊查看，true时为模糊查找
    **********************************************************************************/
    int GetSpecialLeafNum(std::deque<std::string> root_queue, const std::string &child_name,
        const bool fuzzy_search = false);
    /**********************************************************************************
    function: 获取指定层级下，指定节点名的数量
        参数：root_queue：各层级节点名
            child_name：期望计算数量的节点名
            fuzzy_search：对查找的节点名是否进行模糊查看，true时为模糊查找
    **********************************************************************************/
    int GetSpecialNodeNumInOneLevel(std::deque<std::string> root_queue, const std::string &child_name,
        const bool fuzzy_search = false);

    /**********************************************************************************
    function: 遍历整棵树，并获取所有节点信息
        参数：pt：xml节点树
            config_params：保存所有节点信息
    **********************************************************************************/
    void TraverseXmlRoot(boost::property_tree::ptree pt, std::vector<ConfigParam> &config_params);
    /**********************************************************************************
    function: 遍历树中当前层级，并获取该层级下所有节点信息
        参数：pt：xml节点树
            config_params：保存所有节点信息
    **********************************************************************************/
    void TraverseXmlNodeInOneLevel(boost::property_tree::ptree pt, std::vector<ConfigParam> &config_params);

    /**********************************************************************************
    function: 获取指定节点层级下子节点数量
        注意：使用前必须先执行InitConfigFileOperator()进行初始化
        参数：root_queue：指定的各层级节点名
        返回值：子节点数量
    **********************************************************************************/
	size_t GetChildNum(std::deque<std::string> root_queue)
	{
        if(!m_is_initialized)
        {
            return 0;
        }
		if (root_queue.empty())
        {
            root_queue = m_root_queue;
		}

        boost::property_tree::ptree root = *m_root_tree;
        for (std::deque<std::string>::iterator iter = root_queue.begin();
            iter != root_queue.end(); ++iter)
		{
			root = root.get_child(*iter);
		}
		return root.size();
    }

    /**********************************************************************************
    function: 读取xml中的参数值
        注意：使用前必须先执行InitConfigFileOperator()进行初始化
        参数：param_name：参数名
            value_out：用于存放获取到的参数值
            default_value：获取参数值失败时的默认值
            disp_banner：是否打印参数值
            disp_error：是否打印错误信息
        返回值：成功或失败
    **********************************************************************************/
	template <typename T>
    bool GetParam(const std::string &param_name, T &value_out, const T &default_value,
                  const bool disp_banner = false, const bool disp_error = true)
	{
		if (!m_is_initialized)
		{
            if(disp_error)
            {
                std::stringstream ss;
                std::string str("");
                ss << default_value;
                ss >> str;
                std::string node_name("");
                for (std::deque<std::string>::iterator iter = m_root_queue.begin();
                    iter != m_root_queue.end(); ++iter)
                {
                    node_name = node_name + (*iter) + "/";
                }
                node_name += param_name;
                // LOG_ERROR("Read XML node(%s) failed, please initialize first!",
                //           node_name.c_str(), str.c_str());
                value_out = default_value;
                ss.clear();
                ss.str("");
            }
			return false;
		}

        boost::property_tree::ptree root = *m_root_tree;
		try
		{
			if (!m_root_queue.empty())
            {
				for (std::deque<std::string>::iterator iter = m_root_queue.begin();
					iter != m_root_queue.end(); ++iter)
                {
					root = root.get_child(*iter);
				}
            }
		}
		catch (const std::exception &e)
		{
            if(disp_error)
            {
                std::stringstream ss;
                std::string str("");
                ss << default_value;
                ss >> str;
                std::string node_name("");
                for (std::deque<std::string>::iterator iter = m_root_queue.begin();
                    iter != m_root_queue.end(); ++iter)
                {
                    node_name = node_name + (*iter) + "/";
                }
                node_name += param_name;
                // LOG_ERROR("Read XML node(%s) failed(error: %s), using default value(%s)!",
                //           node_name.c_str(), e.what(), str.c_str());
                value_out = default_value;
                ss.clear();
                ss.str("");
            }
			return false;
		}
		
		try
		{
			value_out = root.get<T>(param_name);

			if(disp_banner)
			{
				std::stringstream ss;
				std::string str("");
				ss << value_out;
				ss >> str;
				// LOG_INFO("%s = %s", param_name.c_str(), str.c_str());
                ss.clear();
                ss.str("");
			}
		}
        catch (const std::exception &e)
		{
            if(disp_error)
            {
                std::stringstream ss;
                std::string str("");
                ss << default_value;
                ss >> str;
                std::string node_name("");
                for (std::deque<std::string>::iterator iter = m_root_queue.begin();
                    iter != m_root_queue.end(); ++iter)
                {
                    node_name = node_name + (*iter) + "/";
                }
                node_name += param_name;
                // LOG_ERROR("Get <%s> failed(error: %s), we will use default value(%s)!",
                //           node_name.c_str(), e.what(), str.c_str());
                value_out = default_value;
                ss.clear();
                ss.str("");
            }
			return false;
		}
		return true;
	}

    /**********************************************************************************
    function: 修改或者新增xml中的参数值
        注意：使用前必须先执行InitConfigFileOperator()进行初始化
        参数：param_name：各级节点组合而成，比如：root1.root2.root3
            value：期望修改值
        返回值：成功或失败
    **********************************************************************************/
    template <typename T>
    bool SetParam(const std::string &param_name, const T &value)
    {
        if (!m_is_initialized)
        {
            // LOG_WARNING("Please initialize first!");
            return false;
        }
        try
        {
            m_root_tree->put<T>(param_name, value);
        }
        catch (const std::exception &e)
        {
            // LOG_ERROR("Write XML failed! %s", e.what());
            return false;
        }
        return true;
    }

    /**********************************************************************************
    function: 用于修改或者新增参数文件中的参数，并写入文件
        注意：使用前必须先执行InitConfigFileOperator()进行初始化
        参数：root_que：依次存放从根节点到目标节点的各层节点名
            value：期望修改值
        返回值：0：成功；非0：失败
    **********************************************************************************/
    template <typename T>
    int UpdateParam(std::deque<std::string> root_que, const T &value)
    {
        if (!m_is_initialized)
        {
            // LOG_WARNING("Please initialize first!");
            return 1;
        }
        if (root_que.empty())
        {
            return 2;
        }

        try
        {
            std::string node_name("");
            for (std::deque<std::string>::const_iterator iter = root_que.begin();
                iter != root_que.end(); ++iter)
            {
                node_name = node_name + root_que.front() + SINGLE_DOT_FLAG;
                root_que.pop_front();
            }
            //去掉最后一个"."
            node_name = node_name.substr(0, node_name.length() - 1);

            m_root_tree->put<T>(node_name.c_str(), value);
            WriteXmlFile(m_opened_file_path);//write to xml file
        }
        catch (const std::exception &e)
        {
            // LOG_ERROR("Write XML child node failed! %s", e.what());
            return 3;
        }
        return 0;
    }

    int StringIterToInt(std::basic_string <char>::const_iterator str_Iter)
    {
        char c = *str_Iter;
        std::stringstream ss;
        ss << c;
        int a = 0;
        ss >> a;
        ss.clear();
        ss.str("");
        return a;
    }

    /**********************************************************************************
    function: 将m_root_tree中的内容写入文件
        注意：使用前必须先执行InitConfigFileOperator()进行初始化
        参数：xml_name：文件完整路径
    **********************************************************************************/
	void WriteXmlFile(const std::string &xml_name)
	{
		if (!m_is_initialized)
		{
			// LOG_WARNING("Please initialize first!");
		}

		//set xml format
		boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
		boost::property_tree::write_xml(xml_name, *m_root_tree, std::locale(), settings);
	}

	void WriteStringToFile(std::ofstream &file_stream, const std::string &str_data)
	{
// 		const char *data(str_data.data());
// 		file_stream.write(data, str_data.size());
		file_stream << str_data;
	}

	void WriteCharToFile(std::ofstream &file_stream, const char data)
	{
		file_stream.put(data);
	}

	/*********************************************************************
	function: delete file in directory
	*********************************************************************/
	bool DeleteFile(const std::string &dir)
	{
		int ret = remove(dir.c_str());
		if (ret < 0)
		{
            // LOG_WARNING("Delete %s failed(error: %s)!", dir.c_str(), strerror(errno));
			return false;
		}
		return true;
	}

    /*********************************************************************
    function: Rename file or move file
    *********************************************************************/
    bool RenameFile(const std::string &old_path, const std::string &new_path)
    {
        int ret = rename(old_path.c_str(), new_path.c_str());
        if (ret < 0)
        {
            // LOG_WARNING("Rename %s to %s failed(error: %s)!",
            //             old_path.c_str(), new_path.c_str(), strerror(errno));
            return false;
        }
        return true;
    }

    /*********************************************************************
    function: Make directory
    *********************************************************************/
    int MakeDirectory(const std::string &dir)
    {
        //目录已经存在
        if (0 == ACCESS(dir.c_str(), 0))
        {
//            LOG_INFO("Dir(%s) exists.", dir.c_str());
            return 0;
        }

        std::string tar_dir(dir);
        //确保目录以目录符结尾
        AddDirectoryFlag(tar_dir);

        size_t dir_len = tar_dir.length();
        std::string dir_temp("");
        for (size_t i = 0; i < dir_len; ++i)
        {
            dir_temp += tar_dir[i];
            //不是目录
            if ('\\' != dir_temp[i] && '/' != dir_temp[i])
            {
                continue;
            }
            //目录已经存在
            if (0 == ACCESS(dir_temp.c_str(), 0))
            {
                continue;
            }
            //创建目录
            int ret = MKDIR(dir_temp.c_str());
            if (0 != ret)
            {
                // LOG_ERROR("Make dir(%s) failed(error: %d)!", dir_temp.c_str(), ret);
                return ret;
            }
        }
        return 0;
    }

private:
	/*********************************************************************
	function: boost read xml file
	*********************************************************************/
    bool ReadXmlFile(const std::string &strPath, boost::property_tree::ptree *pt);

private:
    std::string m_file_path, m_opened_file_path;
	std::deque<std::string> m_root_queue;
	boost::property_tree::ptree *m_root_tree;
	std::string m_cur_path;
	bool m_is_initialized;
};

} // namespace SlamCommon
#endif // CONFIG_FILE_OPERATOR_H_
