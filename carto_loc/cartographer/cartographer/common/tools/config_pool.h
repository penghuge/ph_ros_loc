#ifndef CONFIG_POOL_H_
#define CONFIG_POOL_H_

#include "msg_pool.h"

//#define CFG_POOL_NAV350_PARAMS          "nav350"
//#define CFG_POOL_NANOSCAN_PARAMS        "nanoscan"
//#define CFG_POOL_LEIMOUF10_PARAMS       "lei_m_f10"
//#define CFG_POOL_LMSXX_PARAMS           "lmsxx"
#define CFG_POOL_ROBOT_DES              "rob_des"
#define CFG_POOL_OMD30M_PARAMS          "omd30m"
#define CFG_POOL_XINGSONG_PARAMS          "xingsong"
#define CFG_POOL_LOC_INTERFACE          "loc_ifs"
#define CFG_POOL_LASER_OBS              "laser_obs"
#define CFG_POOL_MAINTASK               "main_task"
#define CFG_POOL_SCHEDULE               "schedule"
#define CFG_POOL_MAPPING_IFS            "mapping_ifs"
#define CFG_POOL_NAVI_ALG               "navi_alg"
#define CFG_POOL_NAVI_IFS               "navi_ifs"
#define CFG_POOL_KINEMATICS_MIO         "kine_mio"

class CConfigPool : public CSingletonTemp<CConfigPool>
{
   // 需要将基类声明成友员，以便访问子类的私有构造函数和析构函数
   friend class CSingletonTemp<CConfigPool>;

public:
    CConfigPool(const CConfigPool&)=delete;
    CConfigPool& operator =(const CConfigPool&)= delete;
    ~CConfigPool()
    {
       for(MsgItemMapTypeIter iter = m_data_map.begin();
           iter != m_data_map.end(); ++iter)
       {
           DELETE_PTR(iter->second.msg_cache);
           m_data_map.erase(iter);
       }
    }
    /*********************************************************************
    Function: 新增或者刷新数据类型
       参数： key：消息类型
             data：消息指针，必须是new出来的内存
       返回值：消息池已满：MSG_POOL_INIT_CNT
             成功：消息计数
    *********************************************************************/
    template<typename T>
    unsigned int AddData(const std::string &key, T *data)
    {
       //如果是新增的消息类型，添加对应属性项
       if(m_data_map.end() == m_data_map.find(key))
       {
           MsgItemStru item;
           item.cnt = MSG_POOL_INIT_CNT + 1;
           m_data_map.insert(std::pair<std::string, MsgItemStru>(key, item));
       }

       DeleteMsgCache(key);
       m_data_map[key].msg_cache = new CMsgCache<T>(data, false);
       ++m_data_map[key].cnt;
       return m_data_map[key].cnt;
    }

    /*********************************************************************
    Function: 获取key对应的数据指针
       返回值：复制成功：对应消息缓存的指针
              查找失败：nullptr
    *********************************************************************/
    template<typename T>
    T *GetData(const std::string &key)
    {
        MsgItemMapTypeIter iter = m_data_map.find(key);
        if(m_data_map.end() == iter)
        {
           return nullptr;
        }
        return static_cast<T *>(iter->second.msg_cache->GetData());
    }

    /*********************************************************************
    Function: 复制key对应的数据
       返回值：查找失败：MSG_POOL_INIT_CNT
              复制成功：当前计数值
    *********************************************************************/
    template<typename T>
    unsigned int CloneData(const std::string &key, T &dest)
    {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return MSG_POOL_INIT_CNT;
       }
       dest = *(static_cast<T *>(iter->second.msg_cache->GetData()));
       return m_data_map[key].cnt;
    }

    /*********************************************************************
    Function: 查看消息计数是否更新，可用于判断消息池中的消息数据是否更新
             通过该接口查看消息更新状态，可减少上锁的开销
         返回值：查询成功：对应消息的计数值
                查询失败：MSG_POOL_INIT_CNT
    *********************************************************************/
    unsigned int PeekMsgCnt(const std::string &key)
    {
        MsgItemMapTypeIter it = m_data_map.find(key);
        if(m_data_map.end() == it)
        {
            return MSG_POOL_INIT_CNT;
        }
        return it->second.cnt;
    }
    /*********************************************************************
    Function: 销毁消息，释放内存，并从消息map中去掉消息类型
    *********************************************************************/
    void DestroyMsg(const std::string &key)
    {
        MsgItemMapTypeIter iter = m_data_map.find(key);
        if(m_data_map.end() == iter)
        {
            return;
        }
        DELETE_PTR(iter->second.msg_cache);
        m_data_map.erase(iter);
    }

private:
    CConfigPool()
    {
       m_data_map.clear();
    }

    bool DeleteMsgCache(const std::string &key)
    {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return false;
       }
       DELETE_PTR(iter->second.msg_cache);
       return true;
    }

private:
    MsgItemMapType m_data_map;//每种消息类型对应的消息缓存和属性
};

#endif //!CONFIG_POOL_H_
