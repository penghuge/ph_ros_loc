#include "log_writer.h"

namespace Logging
{
CLogWriter::CLogWriter()
{
}

CLogWriter::~CLogWriter()
{
    //stop to receive logging data
	m_refuse_logging_data = true;

    //print all log to screen or file
//	int i = 0;
	while (!m_read_buffer_ptr->empty() || !m_write_buffer_ptr->empty())
	{
//		std::cout << i++ << std::endl;
		m_log_updated = true;
		m_log_cv.notify_one();
		LOG_SLEEP(10);
	}
    std::unique_lock<std::mutex> log_lock(m_log_mutex);
	m_stop_logging = true;
	m_log_cv.notify_one();
    log_lock.unlock();

    //print all file data to file
    while (!m_read_file_ptr->empty() || !m_write_file_ptr->empty())
    {
        m_file_updated = true;
        m_file_cv.notify_one();
        LOG_SLEEP(10);
    }
    std::unique_lock<std::mutex> file_lock(m_file_mutex);
    m_stop_file_logging = true;
    m_file_cv.notify_one();
    file_lock.unlock();

	DELETE_THREAD(m_write_log_thread);
	m_log_stream.close();

    DELETE_THREAD(m_write_file_thread);
    m_file_stream.close();
}

void CLogWriter::Init(const std::string &log_name, const std::string &log_dir)
{
    //log init
	m_log_name = log_name;
	m_log_dir = log_dir;
	m_write_buffer_ptr = &m_log_stream_buf1;
	m_read_buffer_ptr = &m_log_stream_buf2;
	m_last_update_time = GetCurTimeMS() - m_flush_period;

	m_write_log_thread = new std::thread(&CLogWriter::WriteLogThread, this);

    //record file init
    m_write_file_ptr = &m_file_stream_buf1;
    m_read_file_ptr = &m_file_stream_buf2;
    m_file_last_update_time = GetCurTimeMS() - m_file_flush_period;

    m_write_file_thread = new std::thread(&CLogWriter::WriteRecordFileThread, this);
}

unsigned long CLogWriter::GetCurTimeMS()
{
#ifdef WIN32
	return GetTickCount();
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec * 0.001);
#endif
}

void CLogWriter::PushStreamIntoLoggingBuffer(const std::string &log_str)
{
	if (m_refuse_logging_data)
	{
		return;
	}

	std::unique_lock<std::mutex> lock(m_log_mutex);
	m_write_buffer_ptr->push(log_str);
	unsigned long  now = GetCurTimeMS();
	if (now - m_last_update_time < m_flush_period)
	{
		lock.unlock();
		return;
	}
// 	int diff_t = now - m_last_update_time;
// 	std::stringstream ss;
// 	ss << diff_t;
// 	m_write_buffer_ptr->push("diff time" + ss.str() + "\n");
	m_last_update_time = now;
	m_log_updated = true;
	lock.unlock();
	m_log_cv.notify_one();
}

void CLogWriter::WriteLogThread()
{
	std::queue<std::string> *tmpt_ptr = nullptr;
	m_stop_logging = false;
    int file_index = 1;
    std::string log_name_pre = m_log_dir + m_log_name + UNDERLINE_FLAG;
    std::string cur_log_name = log_name_pre + NumToString(file_index++) + LOG_SUFFIX;

//    std::cout << "CLogWriter::WriteLogThread(): cur_log_name = " << cur_log_name << std::endl;
    m_log_stream.open(cur_log_name, std::ios::out | std::ios::binary | std::ios::app);
    int cnt = 0;
//    std::cout << "CLogWriter::WriteLogThread(): Logging thread has started!" << std::endl;
	while (!m_stop_logging)
	{
        //check log file size
        if(++cnt > 100)
        {
            cnt = 0;
            if(GetFileSize(cur_log_name) > m_log_max_size)
            {
                m_log_stream.close();
                cur_log_name = log_name_pre + NumToString(file_index++) + LOG_SUFFIX;
                m_log_stream.open(cur_log_name, std::ios::out | std::ios::binary | std::ios::app);
            }
        }
		{
			std::unique_lock<std::mutex> lock(m_log_mutex);
			m_log_cv.wait_for(lock, std::chrono::milliseconds(m_flush_period),
				[this]() {return m_log_updated || m_stop_logging; });
			if (m_stop_logging)
			{
				break;
			}
			//swap read and write buffer
			tmpt_ptr = m_read_buffer_ptr;
			m_read_buffer_ptr = m_write_buffer_ptr;
			m_write_buffer_ptr = tmpt_ptr;
			m_log_updated = false;
			lock.unlock();
		}

// 		m_read_buffer_ptr->push("We will write log.....................\n");
		while (!m_read_buffer_ptr->empty())
		{
//			WriteStringToFile(m_log_stream, m_read_buffer_ptr->front());
			m_log_stream << m_read_buffer_ptr->front();
			m_read_buffer_ptr->pop();
		}
		//flush stream buffer
		m_log_stream.flush();
	}
//	std::cout << "Logging thread has shut down!" << std::endl;
}

uint64_t CLogWriter::GetFileSize(const std::string &file_name)
{
    long size = 0;
    std::string file(file_name);

    //print
//    std::string log = "Attempt to read file: " + file;
//    PushStreamIntoBuffer(log);

    FILE *fp = fopen(file.c_str(), "r");
    if(NULL == fp)
    {
        return 0;
    }

#ifdef WIN32
    int fd = _fileno(fp);
#else
	int fd = fileno(fp);
#endif
    struct stat buf;
    fstat(fd, &buf);
    size = (uint64_t)(buf.st_size);

    //print
//    log = "File size: " + NumToString(size);
//    PushStreamIntoBuffer(log);

    fclose(fp);
    return size;
}

void CLogWriter::PushStreamIntoFileBuffer(const std::string &file_str)
{
    if (m_refuse_logging_data)
    {
        return;
    }

    std::unique_lock<std::mutex> lock(m_file_mutex);
    m_write_file_ptr->push(file_str);
    unsigned long  now = GetCurTimeMS();
    if (now - m_file_last_update_time < m_file_flush_period)
    {
        lock.unlock();
        return;
    }
    m_file_last_update_time = now;
    m_file_updated = true;
    lock.unlock();
    m_file_cv.notify_one();
}

void CLogWriter::WriteRecordFileThread()
{
    std::queue<std::string> *tmpt_ptr = nullptr;
    m_stop_file_logging = false;
    std::string file_name = m_log_dir + RECORD_FILE_NAME;

//    std::cout << "CLogWriter::WriteRecordFileThread(): cur_file_name = " << file_name << std::endl;
    m_file_stream.open(file_name, std::ios::out | std::ios::binary | std::ios::app);
//    std::cout << "CLogWriter::WriteRecordFileThread(): Record file thread has started!" << std::endl;
    while (!m_stop_file_logging)
    {
        {
            std::unique_lock<std::mutex> lock(m_file_mutex);
            m_file_cv.wait_for(lock, std::chrono::milliseconds(m_file_flush_period),
                [this]() {return m_file_updated || m_stop_file_logging; });
            if (m_stop_file_logging)
            {
                break;
            }
            //swap read and write buffer
            tmpt_ptr = m_read_file_ptr;
            m_read_file_ptr = m_write_file_ptr;
            m_write_file_ptr = tmpt_ptr;
            m_file_updated = false;
            lock.unlock();
        }

        while (!m_read_file_ptr->empty())
        {
            m_file_stream << m_read_file_ptr->front();
            m_read_file_ptr->pop();
        }
        //flush stream buffer
        m_file_stream.flush();
    }
//    std::cout << "Record file thread has shut down!" << std::endl;
}

void CLogWriter::WriteNamedFile(const std::string &file_name, const std::string &log)
{
    std::string file = m_log_dir + file_name;

//    std::cout << "CLogWriter::WriteRecordFileThread(): cur_file_name = " << file << std::endl;
    m_named_file_stream.open(file, std::ios::out | std::ios::binary | std::ios::app);
    m_named_file_stream << log;
    m_named_file_stream.close();
}


} // namespace Logging