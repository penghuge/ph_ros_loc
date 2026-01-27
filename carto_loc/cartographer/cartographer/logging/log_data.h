#ifndef LOG_DATA_H_
#define LOG_DATA_H_

#include "stdarg.h"
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <limits.h>
#include "time.h"
#include <cstring>

#ifdef WIN32
#include "tchar.h"
#include <process.h>
#ifndef NOMINMAX
#define NOMINMAX					//for std::min() and std::max()
#endif
#include <winsock2.h>
#include <windows.h>
#define getpid _getpid
#else
#include <unistd.h>
#include <sys/time.h>
#endif

namespace Logging
{
#define DELETE_PTR(ptr)             if((ptr) != nullptr){delete (ptr); (ptr) = nullptr;}
#define DELETE_ARRAY_PTR(ptr)       if((ptr) != nullptr){delete[] (ptr); (ptr) = nullptr;}
#define DELETE_THREAD(ptr)          if((ptr) != nullptr){(ptr)->join(); delete (ptr); (ptr) = nullptr;}

#define SET_BIT(x,y)                (x)|=(1<<(y))
#define CLEAR_BIT(x,y)              (x)&=~(1<<(y))
#define REVERSE_BIT(x,y)            (x)^=(1<<(y))
#define GET_BIT(x,y)                (((x)>>(y))&1)

#define LOG_FLUSH_PERIOD            (2000)
#define LOG_FILE_MAX_SIZE			(50000000)

#define LOG_SUFFIX                  (".log")
#define DEBUG_TAG                   ("[DEBUG]")
#define INFO_TAG                    ("[INFO]")
#define WARNING_TAG                 ("[WARNING]")
#define ERROR_TAG                   ("[ERROR]")
#define FATAL_TAG                   ("[FATAL]")

#define BAN_HEADER_FLAG             ("#h")	//ban log header(including type/pid/function/line)
#define BAN_RETURN_FLAG             ("#n")	//ban "\n"
#define RETURN_FLAG                 ("\n")

#define UNDERLINE_FLAG              ("_")
#define MIDLINE_FLAG                ("-")
#define SINGLE_DOT_FLAG             (".")
#define DOUBLE_DOT_FLAG             ("..")

#define COMMA_FLAG                  (",")
#define ASTERISK_FLAG               ("*")

#ifdef WIN32
#define DIR_FLAG                    ("\\")
#define DIR_FLAG_CHAR               ('\\')
#define DIR_INVERSE_FLAG            ("/")
#define DIR_INVERSE_FLAG_CHAR       ('/')
//#define PATH_MAX                    (100)
#define LOG_SLEEP(t)                (Sleep(t))
#else
#define DIR_FLAG                    ("/")
#define DIR_FLAG_CHAR               ('/')
#define DIR_INVERSE_FLAG			("\\")
#define DIR_INVERSE_FLAG_CHAR		('\\')
#define LOG_SLEEP(t)                (usleep(1000 * (t)))
#endif

typedef enum LogLevel
{
	LOGLEVEL_UNKNOWN,
    LOGLEVEL_INFO,
    LOGLEVEL_WARNING,
    LOGLEVEL_ERROR,
    LOGLEVEL_FATAL,
	LOGLEVEL_DEBUG,
	LOGLEVEL_DISABLE
}LogLevel_t;

template <typename T>
std::string NumToString(const T num, int decplaces = 3)
{
    std::ostringstream oss;
    //std::fixed means keep decplaces behind decimal point
    oss << std::fixed << std::setprecision(decplaces) << num;
    return oss.str();
}

} // namespace Logging
#endif // !LOG_H_
