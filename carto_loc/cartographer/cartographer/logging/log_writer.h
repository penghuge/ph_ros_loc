#ifndef LOG_WRITER_H_
#define LOG_WRITER_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "log_data.h"

namespace Logging
{
class CLogWriter
{
#define RECORD_FILE_NAME            "data.txt"

public:
	CLogWriter();
	~CLogWriter();
	void Init(const std::string &log_name, const std::string &log_dir);
    void PushStreamIntoLoggingBuffer(const std::string &log_str);
    void PushStreamIntoFileBuffer(const std::string &file_str);
	void SetFlushPeriod(unsigned long period_ms)
	{
		m_flush_period = period_ms;
	}
    void SetRecordFileFlushPeriod(unsigned long period_ms)
    {
        m_file_flush_period = period_ms;
    }
    //set log file max size, limit to 50Mb, unit: byte
    void SetLogFileMaxSize(const uint64_t size)
    {
		m_log_max_size = size > LOG_FILE_MAX_SIZE ? LOG_FILE_MAX_SIZE : size;
    }
    uint64_t GetFileSize(const std::string &file_name);
    void WriteNamedFile(const std::string &file_name, const std::string &log);
private:
    void WriteLogThread();
    void WriteRecordFileThread();
	unsigned long GetCurTimeMS();
	void WriteStringToFile(std::ofstream &file_stream, const std::string &str_data)
	{
		const char *data(str_data.data());
		file_stream.write(data, str_data.size());
	}
	void WriteCharToFile(std::ofstream &file_stream, const char data)
	{
		file_stream.put(data);
    }

private:
    bool m_refuse_logging_data = false;

    //log
	std::string m_log_name, m_log_dir;

	std::queue<std::string> *m_write_buffer_ptr = nullptr;
	std::queue<std::string> *m_read_buffer_ptr = nullptr;
	std::queue<std::string> m_log_stream_buf1;
	std::queue<std::string> m_log_stream_buf2;

	std::thread *m_write_log_thread = nullptr;
	std::mutex m_log_mutex;
    std::condition_variable m_log_cv;
    bool m_log_updated = false;
    bool m_stop_logging = true;

	std::ofstream m_log_stream;
	unsigned long m_last_update_time;
    unsigned long m_flush_period = LOG_FLUSH_PERIOD;	//unit: ms
    uint64_t m_log_max_size = 20000000; //default size is 20Mb

    //record file
    std::queue<std::string> *m_write_file_ptr = nullptr;
    std::queue<std::string> *m_read_file_ptr = nullptr;
    std::queue<std::string> m_file_stream_buf1;
    std::queue<std::string> m_file_stream_buf2;

    std::thread *m_write_file_thread = nullptr;
    std::mutex m_file_mutex;
    std::condition_variable m_file_cv;
    bool m_file_updated = false;
    bool m_stop_file_logging = true;

    //记录data.txt文件
    std::ofstream m_file_stream;
    unsigned long m_file_last_update_time;
    unsigned long m_file_flush_period = LOG_FLUSH_PERIOD;	//unit: ms

    //记录指定文件名的日志
    std::ofstream m_named_file_stream;
};

} // namespace Logging
#endif // !LOG_H_
