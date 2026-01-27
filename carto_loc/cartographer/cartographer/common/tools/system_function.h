#ifndef SYSTEM_FUNCTION_H_
#define SYSTEM_FUNCTION_H_

#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "cartographer/logging/log.h"

#ifdef ENABLE_QNX_OS
typedef void (*QNXRTPthreadType)(union sigval);
#endif

#ifdef ENABLE_QNX_OS
inline int TimerPthreadInit(timer_t &timerid, QNXRTPthreadType fun, const uint64_t period_s, const uint64_t period_ns,
                              void *sival_ptr = nullptr, const int sival_int = 0)
{
    struct sigevent evp;
    memset(&evp, 0, sizeof(struct sigevent));   //清零初始化

    evp.sigev_value.sival_int = sival_int;
    if(nullptr != sival_ptr)
    {
        evp.sigev_value.sival_ptr = sival_ptr;
    }
    evp.sigev_notify = SIGEV_THREAD;        //线程通知的方式，派驻新线程
    evp.sigev_notify_function = fun;   //线程函数地址

    int ret = timer_create(CLOCK_REALTIME, &evp, &timerid);
    if(ret)
    {
        LOG_ERROR("Timer pthread create failed(error: %d)!", ret);
    }

    struct itimerspec it;
    it.it_interval.tv_sec = period_s;   //执行周期
    it.it_interval.tv_nsec = period_ns;
    it.it_value.tv_sec = period_s;      //起始时间
    it.it_value.tv_nsec = period_ns;

    ret = timer_settime(timerid, 0, &it, NULL);
    if(ret)
    {
        LOG_ERROR("Timer set time failed(error: %d)!", ret);
    }

    return ret;
}
#endif

inline int RunSystemCmd(const char* cmd)
{
    pid_t status;
    status = system(cmd);
    if (-1 == status)
    {
//        printf("system error!");
        return 1;
    }
//    printf("exit status value = [0x%x]\n", status);

    if (WIFEXITED(status))
    {
        if (0 != WEXITSTATUS(status))
        {
//            printf("run shell script fail, script exit code: %d\n", WEXITSTATUS(status));
            return 2;
        }
//        printf("run shell script successfully.\n");
    }
//    printf("exit status = [%d]\n", WEXITSTATUS(status));
    return 0;
}

inline int RebootSystem()
{
#ifdef ENABLE_QNX_OS
    return RunSystemCmd("shutdown");
#else
    return RunSystemCmd("reboot");
#endif
}

inline int PoweroffSystem()
{
#ifdef ENABLE_QNX_OS
    return RunSystemCmd("shutdown -b");
#else
    return RunSystemCmd("poweroff");
#endif
}

#endif  // SYSTEM_FUNCTION_H_
