#ifndef MSG_POOL_H_
#define MSG_POOL_H_

#include <map>
#include <mutex>
#include <condition_variable>
#include "cartographer/common/const_value.h"
#include "template_class.h"

#define MSG_POOL_INIT_CNT               0

#define MSG_POOL_LASER_SCAN             "scan"
#define MSG_POOL_LASER_POSE             "laser_pos"
#define MSG_POOL_IMU                    "imu"
#define MSG_POOL_ODOMETRY               "odom"
#define MSG_POOL_OCCUPANCY_GRID         "occu_grid"
#define MSG_POOL_REFLECTOR_MAP          "refle_map"
#define MSG_POOL_REFLECTOR              "refle"
#define MSG_POOL_TRAJECTORY             "traj"
#define MSG_POOL_ERROR_LIST             "err_list"
#define MSG_POOL_INIT_STATIONS          "init_sta"

class CMsgCacheBase
{
public:
    CMsgCacheBase() {}
    virtual ~CMsgCacheBase() {}
    virtual void *GetData() = 0;
};

template <typename T>
class CMsgCache : public CMsgCacheBase
{
public:
    CMsgCache(T *msg, const bool is_array)
        : m_data(msg), m_is_array(is_array)
    {}
    CMsgCache(const CMsgCache &other)
    {
        m_data = other.m_data;
    }
    virtual ~CMsgCache()
    {
        if(m_is_array)
        {
            DELETE_ARRAY_PTR(m_data);
        }
        else
        {
            DELETE_PTR(m_data);
        }
    }

    /*********************************************************************
    Function: 获取数据
    *********************************************************************/
    virtual void *GetData() override
    {
        return m_data;
    }

private:
    T *m_data = nullptr;
    bool m_is_array = false;
};

//激光数据量较大，频繁进行内存操作，开销较大，专门写一个消息类型
//数据更新通过同一内存中数据刷新实现，而不是销毁原来内存后，再新开辟内存
class CLaserScanMsgCache : public CMsgCacheBase
{
public:
    CLaserScanMsgCache(SlamCommon::SlamLaserScanData *msg) : m_data(msg) {}
    CLaserScanMsgCache(const CLaserScanMsgCache &other) : m_data(other.m_data) {}
    virtual ~CLaserScanMsgCache()
    {
        DELETE_PTR(m_data);
    }
    virtual void *GetData() override
    {
        return nullptr;
    }
    SlamCommon::SlamLaserScanData *GetLaserScanData()
    {
        return m_data;
    }
    /*********************************************************************
    Function: 刷新数据
    *********************************************************************/
    void UpdateData(SlamCommon::SlamLaserScanData *data)
    {
        *m_data = *data;
    }

private:
    SlamCommon::SlamLaserScanData *m_data = nullptr;
};

struct MsgItemStru
{
   unsigned int cnt = MSG_POOL_INIT_CNT;    //消息计数值，计数值变化时，表示消息已经更新
   int mutex_index = 0;                     //锁的下标
   int cv_index = 0;                        //通知器的下标
   CMsgCacheBase *msg_cache = nullptr;      //消息缓存
};

typedef std::map<std::string, MsgItemStru>              MsgItemMapType;
typedef std::map<std::string, MsgItemStru>::iterator    MsgItemMapTypeIter;

#define MSG_POOL_MAX_SIZE       100

class CMsgPool : public CSingletonTemp<CMsgPool>
{
   // 需要将基类声明成友员，以便访问子类的私有构造函数和析构函数
   friend class CSingletonTemp<CMsgPool>;

public:
    CMsgPool(const CMsgPool&)=delete;
    CMsgPool& operator =(const CMsgPool&)= delete;
    ~CMsgPool()
    {
       for(MsgItemMapTypeIter iter = m_data_map.begin();
           iter != m_data_map.end(); ++iter)
       {
           std::unique_lock<std::mutex> lock(GetMutex(iter->first));
           DELETE_PTR(iter->second.msg_cache);
           m_data_map.erase(iter);
       }
    }
    /*********************************************************************
    Function: 新增或者刷新数据类型
       参数： key：消息类型
             data：消息指针，必须是new出来的内存
             is_array：是否是数组，主要用于析构时对数组的特殊析构方式
       返回值：消息池已满：MSG_POOL_INIT_CNT
             成功：消息计数
    *********************************************************************/
    template<typename T>
    unsigned int AddData(const std::string &key, T *data, const bool is_array)
    {
       //如果消息池中消息数量已经达到了最大值，则不能再添加消息了，如果还需要添加，则加大 MSG_POOL_MAX_SIZE
       if(m_data_map.size() >= MSG_POOL_MAX_SIZE)
       {
        //    LOG_ERROR("Msg pool is full(size=%d)!", m_data_map.size());
           return MSG_POOL_INIT_CNT;
       }
       //如果是新增的消息类型，添加对应属性项
       if(m_data_map.end() == m_data_map.find(key))
       {
           MsgItemStru item;
           item.cnt = MSG_POOL_INIT_CNT + 1;
           item.cv_index = m_data_map.size();
           item.mutex_index = m_data_map.size();
           m_data_map.insert(std::pair<std::string, MsgItemStru>(key, item));
       }

       std::unique_lock<std::mutex> lock(GetMutex(key));
       DeleteMsgCache(key);
       m_data_map[key].msg_cache = new CMsgCache<T>(data, is_array);
       ++m_data_map[key].cnt;
       lock.unlock();
       GetCv(key).notify_all();
       return m_data_map[key].cnt;
    }

    /*********************************************************************
    Function: 不上锁，新增或者刷新数据类型
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定目前没有用户读取
       参数： key：消息类型
             data：消息指针，必须是new出来的内存
             is_array：是否是数组，主要用于析构时对数组的特殊析构方式
       返回值：消息池已满：MSG_POOL_INIT_CNT
             成功：消息计数
    *********************************************************************/
    template<typename T>
    unsigned int AddDataUnlock(const std::string &key, T *data, const bool is_array)
    {
       //如果消息池中消息数量已经达到了最大值，则不能再添加消息了，如果还需要添加，则加大 MSG_POOL_MAX_SIZE
       if(m_data_map.size() >= MSG_POOL_MAX_SIZE)
       {
        //    LOG_ERROR("Msg pool is full(size=%d)!", m_data_map.size());
           return MSG_POOL_INIT_CNT;
       }
       //如果是新增的消息类型，添加对应属性项
       if(m_data_map.end() == m_data_map.find(key))
       {
           MsgItemStru item;
           item.cnt = MSG_POOL_INIT_CNT + 1;
           item.cv_index = m_data_map.size();
           item.mutex_index = m_data_map.size();
           m_data_map.insert(std::pair<std::string, MsgItemStru>(key, item));
       }

       DeleteMsgCache(key);
       m_data_map[key].msg_cache = new CMsgCache<T>(data, is_array);
       ++m_data_map[key].cnt;
       GetCv(key).notify_all();
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
        std::unique_lock<std::mutex> lock(GetMutex(key));
        return static_cast<T *>(iter->second.msg_cache->GetData());
    }

    /*********************************************************************
    Function: 不上锁，获取key对应的数据指针
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定不会再更新的情况
       返回值：复制成功：对应消息缓存的指针
              查找失败：nullptr
    *********************************************************************/
    template<typename T>
    T *GetDataUnlock(const std::string &key)
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
        std::unique_lock<std::mutex> lock(GetMutex(key));
        dest = *(static_cast<T *>(iter->second.msg_cache->GetData()));
        return m_data_map[key].cnt;
    }

    /*********************************************************************
    Function: 不上锁，复制key对应的数据
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定不会再更新的情况
       返回值：查找失败：MSG_POOL_INIT_CNT
              复制成功：当前计数值
    *********************************************************************/
    template<typename T>
    unsigned int CloneDataUnlock(const std::string &key, T &dest)
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
    Function: 直到key对应的数据更新，执行复制操作，
        返回值：查找失败：MSG_POOL_INIT_CNT
               复制成功：当前计数值
    *********************************************************************/
    template<typename T>
    unsigned int CloneDataUntilUpdate(const unsigned int cnt, const std::string &key, T &dest)
    {
        if(m_data_map.end() == m_data_map.find(key))
        {
            return MSG_POOL_INIT_CNT;
        }

        std::unique_lock<std::mutex> lock(GetMutex(key));
        GetCv(key).wait(lock, [&, this]()
        {
           if(m_wake_up_all_cv)
           {
               return true;
           }
           if(cnt == m_data_map[key].cnt)
           {
               return false;
           }
           return true;
        });
        //强制唤醒等待
        if(m_wake_up_all_cv)
        {
           return m_data_map[key].cnt;
        }
        dest = *(static_cast<T *>(m_data_map[key].msg_cache->GetData()));
        return m_data_map[key].cnt;
    }

    /*********************************************************************
    Function: 设置所有cv的唤醒状态，一般用于析构时退出cv.wait
         参数：true：cv不再等待
              false：cv根据条件执行等待
    *********************************************************************/
    void ChangeAllCvWakeUpState(const bool wake_up)
    {
        m_wake_up_all_cv = wake_up;
        if(!wake_up)
        {
            return;
        }

        for(MsgItemMapTypeIter iter = m_data_map.begin();
            iter != m_data_map.end(); ++iter)
        {
            GetCv(iter->first).notify_all();
        }
    }

    /*********************************************************************
    Function: 设置特定cv的唤醒状态，一般用于析构时退出cv.wait
         参数：true：cv不再等待
              false：cv根据条件执行等待
    *********************************************************************/
    void ChangeDesignatedCvWakeUpState(const std::string &key, const bool wake_up)
    {
        if(m_data_map.end() == m_data_map.find(key))
        {
            return;
        }
        m_wake_up_all_cv = wake_up;
        if(!wake_up)
        {
            return;
        }

        GetCv(key).notify_all();
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
    CMsgPool()
    {
       m_data_map.clear();
    }
    std::mutex &GetMutex(const std::string &key)
    {
       return m_msg_mutex[m_data_map[key].mutex_index];
    }
    std::condition_variable &GetCv(const std::string &key)
    {
       return m_msg_cv[m_data_map[key].cv_index];
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
    std::mutex m_msg_mutex[MSG_POOL_MAX_SIZE];//数据锁，如果消息池中消息类型数量超过该数值，则需要加大该值
    std::condition_variable m_msg_cv[MSG_POOL_MAX_SIZE];//数据通知器
    MsgItemMapType m_data_map;//每种消息类型对应的消息缓存和属性
    bool m_wake_up_all_cv = false;//是否保持所有cv的唤醒状态，一般用于析构时退出cv.wait
};

class CLaserScanMsgPool : public CSingletonTemp<CLaserScanMsgPool>
{
   // 需要将基类声明成友员，以便访问子类的私有构造函数和析构函数
   friend class CSingletonTemp<CLaserScanMsgPool>;

public:
   CLaserScanMsgPool(const CLaserScanMsgPool&)=delete;
   CLaserScanMsgPool& operator =(const CLaserScanMsgPool&)= delete;
   ~CLaserScanMsgPool()
   {
       for(MsgItemMapTypeIter iter = m_data_map.begin();
           iter != m_data_map.end(); ++iter)
       {
           std::unique_lock<std::mutex> lock(GetMutex(iter->first));
           DELETE_PTR(iter->second.msg_cache);
           m_data_map.erase(iter);
       }
   }

   /*********************************************************************
   Function: 新增或者刷新laser scan数据
       参数： key：消息类型
             data：消息指针，必须是new出来的内存
       返回值：消息池已满：MSG_POOL_INIT_CNT
             成功：消息计数
   *********************************************************************/
   unsigned int AddData(const std::string &key, SlamCommon::SlamLaserScanData *data)
   {
       //如果消息池中消息数量已经达到了最大值，则不能再添加消息了，如果还需要添加，则加大 MSG_POOL_MAX_SIZE
      if(m_data_map.size() >= MSG_POOL_MAX_SIZE)
      {
        //   LOG_ERROR("Laser scan msg pool is full(size=%d)!", m_data_map.size());
          return MSG_POOL_INIT_CNT;
      }
      //如果是新增的消息类型，添加对应属性项
      if(m_data_map.end() == m_data_map.find(key))
      {
          MsgItemStru item;
          item.cnt = MSG_POOL_INIT_CNT + 1;
          item.cv_index = m_data_map.size();
          item.mutex_index = m_data_map.size();
          m_data_map.insert(std::pair<std::string, MsgItemStru>(key, item));
      }

      std::unique_lock<std::mutex> lock(GetMutex(key));
      DeleteMsgCache(key);
      m_data_map[key].msg_cache = new CLaserScanMsgCache(data);
      ++m_data_map[key].cnt;
      lock.unlock();
      GetCv(key).notify_all();
      return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 不上锁，新增或者刷新数据类型
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定目前没有用户读取
       参数： key：消息类型
             data：消息指针，必须是new出来的内存
       返回值：消息池已满：MSG_POOL_INIT_CNT
             成功：消息计数
   *********************************************************************/
   unsigned int AddDataUnlock(const std::string &key, SlamCommon::SlamLaserScanData *data)
   {
       //如果消息池中消息数量已经达到了最大值，则不能再添加消息了，如果还需要添加，则加大 MSG_POOL_MAX_SIZE
      if(m_data_map.size() >= MSG_POOL_MAX_SIZE)
      {
        //   LOG_ERROR("Laser scan msg pool is full(size=%d)!", m_data_map.size());
          return MSG_POOL_INIT_CNT;
      }
      //如果是新增的消息类型，添加对应属性项
      if(m_data_map.end() == m_data_map.find(key))
      {
          MsgItemStru item;
          item.cnt = MSG_POOL_INIT_CNT + 1;
          item.cv_index = m_data_map.size();
          item.mutex_index = m_data_map.size();
          m_data_map.insert(std::pair<std::string, MsgItemStru>(key, item));
      }

      DeleteMsgCache(key);
      m_data_map[key].msg_cache = new CLaserScanMsgCache(data);
      ++m_data_map[key].cnt;
      GetCv(key).notify_all();
      return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 获取key对应的激光数据指针
       返回值：复制成功：对应消息缓存的指针
              查找失败：nullptr
   *********************************************************************/
   SlamCommon::SlamLaserScanData *GetData(const std::string &key)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return nullptr;
       }
       std::unique_lock<std::mutex> lock(GetMutex(key));
       return static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)
                                                 ->GetLaserScanData();
   }

   /*********************************************************************
   Function: 不上锁，获取key对应的数据指针
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定不会再更新的情况
       返回值：复制成功：对应消息缓存的指针
              查找失败：nullptr
   *********************************************************************/
   SlamCommon::SlamLaserScanData *GetDataUnlock(const std::string &key)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return nullptr;
       }
       return static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)
                                                 ->GetLaserScanData();
   }

   /*********************************************************************
   Function: 复制key对应的激光数据，
       返回值：查找失败：MSG_POOL_INIT_CNT
              复制成功：当前计数值
   *********************************************************************/
   unsigned int CloneData(const std::string &key, SlamCommon::SlamLaserScanData &dest)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return MSG_POOL_INIT_CNT;
       }
       std::unique_lock<std::mutex> lock(GetMutex(key));
       dest = *(static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)
                                                    ->GetLaserScanData());
       return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 不上锁，复制key对应的激光数据
              注：（1）不上锁时速度更快，但是用户需要自行判断是否会产生内存冲突
                 （2）一般可用于数据量非常小，或者该消息可确定不会再更新的情况
       返回值：查找失败：MSG_POOL_INIT_CNT
              复制成功：当前计数值
   *********************************************************************/
   unsigned int CloneDataUnlock(const std::string &key, SlamCommon::SlamLaserScanData &dest)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return MSG_POOL_INIT_CNT;
       }
       dest = *(static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)
                                                    ->GetLaserScanData());
       return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 等待key对应的激光数据更新，执行复制操作
        返回值：查找失败：MSG_POOL_INIT_CNT
               复制成功：当前计数值
   *********************************************************************/
   unsigned int CloneDataUntilUpdate(const unsigned int cnt, const std::string &key,
                                     SlamCommon::SlamLaserScanData &dest)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return MSG_POOL_INIT_CNT;
       }

       std::unique_lock<std::mutex> lock(GetMutex(key));
       GetCv(key).wait(lock, [&, this]()
       {
           if(m_wake_up_all_cv)
           {
               return true;
           }
           if(cnt == m_data_map[key].cnt)
           {
               return false;
           }
           return true;
       });

       //强制唤醒等待
       if(m_wake_up_all_cv)
       {
           return m_data_map[key].cnt;
       }
       dest = *(static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)
                                                    ->GetLaserScanData());
       return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 更新key对应的激光数据，查找失败返回false
       参数：key：消息类型
            data：内存只需要开辟一次，后面不会销毁，可以一直用
            注意：通过更新数据内存的方式实现刷新，是提升效率的关键
       返回值：查询失败：MSG_POOL_INIT_CNT
              成功：消息计数
   *********************************************************************/
   unsigned int UpdateData(const std::string &key, SlamCommon::SlamLaserScanData *data)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return MSG_POOL_INIT_CNT;
       }
       std::unique_lock<std::mutex> lock(GetMutex(key));
       static_cast<CLaserScanMsgCache *>(iter->second.msg_cache)->UpdateData(data);
       ++m_data_map[key].cnt;
       lock.unlock();
       GetCv(key).notify_all();
       return m_data_map[key].cnt;
   }

   /*********************************************************************
   Function: 设置所有cv的唤醒状态，一般用于析构时退出cv.wait
        参数：true：cv不再等待
             false：cv根据条件执行等待
   *********************************************************************/
   void ChangeAllCvWakeUpState(const bool wake_up)
   {
       m_wake_up_all_cv = wake_up;
       if(!wake_up)
       {
           return;
       }

       for(MsgItemMapTypeIter iter = m_data_map.begin();
           iter != m_data_map.end(); ++iter)
       {
           GetCv(iter->first).notify_all();
       }
   }

   /*********************************************************************
   Function: 设置特定cv的唤醒状态，一般用于析构时退出cv.wait
        参数：true：cv不再等待
             false：cv根据条件执行等待
   *********************************************************************/
   void ChangeDesignatedCvWakeUpState(const std::string &key, const bool wake_up)
   {
       if(m_data_map.end() == m_data_map.find(key))
       {
           return;
       }
       m_wake_up_all_cv = wake_up;
       if(!wake_up)
       {
           return;
       }

       GetCv(key).notify_all();
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
       DeleteMsgCache(key);
   }

private:
   CLaserScanMsgPool()
   {
       m_data_map.clear();
   }
   std::mutex &GetMutex(const std::string &key)
   {
      return m_msg_mutex[m_data_map[key].mutex_index];
   }
   std::condition_variable &GetCv(const std::string &key)
   {
      return m_msg_cv[m_data_map[key].cv_index];
   }
   bool DeleteMsgCache(const std::string &key)
   {
       MsgItemMapTypeIter iter = m_data_map.find(key);
       if(m_data_map.end() == iter)
       {
           return false;
       }
       DELETE_PTR(iter->second.msg_cache);
       m_data_map.erase(iter);
       return true;
   }

private:
   std::mutex m_msg_mutex[MSG_POOL_MAX_SIZE];//数据锁，如果消息池中消息类型数量超过该数值，则需要加大该值
   std::condition_variable m_msg_cv[MSG_POOL_MAX_SIZE];//数据通知器
   MsgItemMapType m_data_map;//每种消息类型对应的消息缓存和属性
   bool m_wake_up_all_cv = false;//是否保持所有cv的唤醒状态，一般用于析构时退出cv.wait
};

#endif //!MSG_POOL_H_
