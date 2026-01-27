#include "config_file_operator.h"

namespace SlamCommon
{
CConfigFileOperator::CConfigFileOperator(const std::string &cur_path)
	: m_cur_path(cur_path), m_is_initialized(false)
{
	m_root_tree = new boost::property_tree::ptree();
    m_root_queue.clear();
}

CConfigFileOperator::~CConfigFileOperator()
{
	m_is_initialized = false;
	DELETE_PTR(m_root_tree);
}

/**********************************************************************************
function: set the directory name and module name, before reading xml
**********************************************************************************/
void CConfigFileOperator::SetNodeDirectory(const std::string &node_dir)
{
	if (node_dir.substr(0, m_cur_path.length()) != m_cur_path)
	{
        LOG_INFO("is cur_path： %s", m_cur_path.c_str());
		m_file_path = m_cur_path;
	}
	else
	{
		m_file_path.clear();
	}

	if ("" != node_dir)
	{
        m_file_path = m_file_path + node_dir;
	}

    AddDirectoryFlag(m_file_path);
}

/**********************************************************************************
function: set xml file name and root of the xml file, before reading xml
**********************************************************************************/
bool CConfigFileOperator::InitConfigFileOperator(std::string file_name, std::string root_name)
{
	m_is_initialized = false;
	m_root_tree->clear();

    m_opened_file_path = m_file_path + file_name;
    if (XML_ID != m_opened_file_path.substr(m_opened_file_path.length() - 4,
                                            m_opened_file_path.length()))
	{
        m_opened_file_path += XML_ID;
	}
//    LOG_INFO("opened file path = %s", m_opened_file_path.c_str());
    if (!ReadXmlFile(m_opened_file_path, m_root_tree))//read xml file
    {
        return false;
    }
    if (nullptr == m_root_tree)
    {
        LOG_WARNING("Boost read XML file tree is empty!");
        return false;
    }

    //whether xml has a root node
    m_root_queue.clear();
    if ("" != root_name)
    {
        m_root_queue.push_back(root_name);
    }

    m_is_initialized = true;
    return true;
}

/**********************************************************************************
function: set xml file name and root of the xml file, before reading xml
**********************************************************************************/
bool CConfigFileOperator::InitConfigFileOperator(std::string file_name,
	std::deque<std::string> root_queue)
{
	m_is_initialized = false;
	m_root_tree->clear();

    m_opened_file_path = m_file_path + file_name;
    if (XML_ID != m_opened_file_path.substr(m_opened_file_path.length() - 4, m_opened_file_path.length()))
	{
        m_opened_file_path += XML_ID;
	}

    if (ReadXmlFile(m_opened_file_path, m_root_tree))//read xml file
	{
		if (nullptr == m_root_tree)
		{
			LOG_WARNING("Boost read XML file tree is empty!");
			return false;
		}
		m_root_queue = root_queue;		
		m_is_initialized = true;
		return true;
	}

    LOG_WARNING("Read %s failed!", m_opened_file_path.c_str());
    return false;
}

bool CConfigFileOperator::GetChildrenAndValus(const std::string &file_path,
    std::vector<ConfigParam> &config_params)
{
    m_root_tree->clear();
    if (ReadXmlFile(file_path, m_root_tree))//read xml file
    {
        TraverseXmlRoot(*m_root_tree, config_params);
        return true;
    }
    return false;
}

int CConfigFileOperator::GetSpecialLeafNum(std::deque<std::string> root_queue,
    const std::string &child_name, const bool fuzzy_search)
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
    try
    {
        for (std::deque<std::string>::const_iterator iter = root_queue.begin();
             iter != root_queue.end(); ++iter)
        {
            root = root.get_child(*iter);
        }
    }
    catch(const std::exception &)
    {
        return 0;
    }

    std::vector<ConfigParam> config_params;
    TraverseXmlRoot(root, config_params);
    int child_num = 0, index = 0;
    for (std::vector<ConfigParam>::iterator iter = config_params.begin();
         iter != config_params.end(); ++iter)
    {
        if(fuzzy_search)
        {
            index = iter->node_name.find(child_name);
            if(index >= 0)
            {
                ++child_num;
            }
        }
        else
        {
            if(child_name == iter->node_name)
            {
                ++child_num;
            }
        }
    }
    return child_num;
}

int CConfigFileOperator::GetSpecialNodeNumInOneLevel(std::deque<std::string> root_queue,
    const std::string &child_name, const bool fuzzy_search)
{
    if (root_queue.empty())
    {
        if(!m_is_initialized)
        {
            return 0;
        }
        root_queue = m_root_queue;
    }

    boost::property_tree::ptree root = *m_root_tree;
    try
    {
        for (std::deque<std::string>::const_iterator iter = root_queue.begin();
             iter != root_queue.end(); ++iter)
        {
            root = root.get_child(*iter);
        }
    }
    catch(const std::exception &)
    {
        return 0;
    }

    std::vector<ConfigParam> config_params;
    TraverseXmlNodeInOneLevel(root, config_params);
    int child_num = 0, index = 0;
    for (std::vector<ConfigParam>::iterator iter = config_params.begin();
         iter != config_params.end(); ++iter)
    {
        if(fuzzy_search)
        {
            index = iter->node_name.find(child_name);
            if(index >= 0)
            {
                ++child_num;
            }
        }
        else
        {
            if(child_name == iter->node_name)
            {
                ++child_num;
            }
        }
    }
    return child_num;
}

void CConfigFileOperator::TraverseXmlRoot(boost::property_tree::ptree pt,
    std::vector<ConfigParam> &config_params)
{
    ConfigParam cfg_param;
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v1, pt)
    {
        if (v1.second.empty())
        {
            if("<xmlcomment>" == v1.first)
            {
                cfg_param.comment = v1.second.data();
                continue;
            }
            cfg_param.node_name = v1.first;
            cfg_param.value = v1.second.data();
            config_params.push_back(cfg_param);
//            LOG_INFO("%s = %s, comment: %s", cfg_param.node_name.c_str(), cfg_param.value.c_str(),
//                     cfg_param.comment.c_str());
            cfg_param.comment.clear();
        }
        else
        {
//            LOG_INFO("New node: %s", v1.first.c_str());
            TraverseXmlRoot(v1.second, config_params);
        }
    }
}

void CConfigFileOperator::TraverseXmlNodeInOneLevel(boost::property_tree::ptree pt,
    std::vector<ConfigParam> &config_params)
{
    ConfigParam cfg_param;
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v1, pt)
    {
        if("<xmlcomment>" == v1.first)
        {
            cfg_param.comment = v1.second.data();
            continue;
        }
        cfg_param.node_name = v1.first;
        cfg_param.value = v1.second.data();
        config_params.push_back(cfg_param);
//        LOG_INFO("%s = %s, comment: %s", cfg_param.node_name.c_str(), cfg_param.value.c_str(),
//                 cfg_param.comment.c_str());
        cfg_param.comment.clear();
    }
}

bool CConfigFileOperator::ReadXmlFile(const std::string &strPath, boost::property_tree::ptree *pt)
{
    //check file exists or not
    std::ifstream infile;
    infile.open(strPath.c_str());
    // LOG_INFO("cesi_2_18_xml path = %s", strPath.c_str());
    if (!infile.is_open())
    {
        LOG_ERROR("Open %s failed!", strPath.c_str());
        return false;
    }
    LOG_INFO("Open %s success!", strPath.c_str());
    infile.close();

//    boost::filesystem::path filepath(strPath);
//    if (!boost::filesystem::exists(filepath))
//    {
//        return false;
//    }

    /*
     * const int boost::property_tree::xml_parser::no_concat_text = 1; //不连接文字
     * const int boost::property_tree::xml_parser::no_comments = 2; //去除注释
     * const int boost::property_tree::xml_parser::trim_whitespace=4;//去除多余的换行和空格等
    */
    //读取xml，并去除多余的换行和空格等
    boost::property_tree::read_xml(strPath, *pt, boost::property_tree::xml_parser::trim_whitespace);
    return true;
}

} // namespace SlamCommon
