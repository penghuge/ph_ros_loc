#ifndef ERROR_CODE_HANDLER_H_
#define ERROR_CODE_HANDLER_H_

#include "const_value.h"

namespace SlamCommon {

inline size_t ComputeErrorSize(ErrorType *que, const int que_size)
{
	int i = 0;
    for (; i < que_size; ++i)
	{
		if (NO_ERROR_CODE == que[i])
		{
			break;
		}
	}
	return (size_t)i;
}

/**********************************************************************************
function: Insert a error code to the destinated error code queue,
    need the destinated queue size
**********************************************************************************/
inline void ClearErrorCodeQueue(ErrorType *dest_que, const int dest_que_size)
{
    memset(dest_que, NO_ERROR_CODE, dest_que_size);
}

/**********************************************************************************
function: Insert a error code to the destinated error code queue,
    need the destinated queue size
**********************************************************************************/
inline int InsertErrorCode(ErrorType *dest_que, const int dest_que_size,
    const ErrorType err_code, const int offset = 0)
{
    if (offset < 0 || NO_ERROR_CODE == err_code)
	{
		return -1;
	}
    for (int i = offset; i < dest_que_size; ++i)
	{
		if (NO_ERROR_CODE == dest_que[i] || err_code == dest_que[i])
		{
			dest_que[i] = err_code;
			return i;
		}
	}
	return -1;
}

/**********************************************************************************
function: Insert a error code queue to the destinated error code queue,
    need the destinated queue size and the queue wanted to insert
**********************************************************************************/
inline int InsertErrorCode(ErrorType *dest_que, const int dest_que_size,
    const ErrorType *const err_code_que, const int err_code_que_size)
{
	for (int i = 0; i < err_code_que_size; ++i)
	{
		if (NO_ERROR_CODE == err_code_que[i])
		{
			return ComputeErrorSize(dest_que, dest_que_size);
		}
        InsertErrorCode(dest_que, dest_que_size, err_code_que[i]);
	}
	return ComputeErrorSize(dest_que, dest_que_size);
}

/**********************************************************************************
function: Erase the error code in the queue
**********************************************************************************/
inline void EraseErrorCode(ErrorType *que, const int que_size, const ErrorType err_code, const int offset = 0)
{
	if (offset < 0)
	{
		return;
	}

    for (int i = offset; i < que_size; ++i)
	{
		if (NO_ERROR_CODE == que[i] || err_code == que[i])
		{
			que[i] = NO_ERROR_CODE;
			return;
		}
	}
}

/**********************************************************************************
function: If found the error code, return the index of it in the error code queue,
    otherwise, return -1.
**********************************************************************************/
inline int FindErrorCode(ErrorType *que, const int que_size, const ErrorType err_code)
{
    if (NO_ERROR_CODE != err_code)
    {
        for (int i = 0; i < que_size; ++i)
        {
            if (NO_ERROR_CODE == que[i])
            {
                break;
            }
            if (err_code == que[i])
            {
                return i;
            }
        }
    }
    return -1;
}

/**********************************************************************************
function: Check whether error count has reached limit threshold
    if condition is true, return true, otherwise return false
**********************************************************************************/
inline bool CheckErrorState(const bool condition, int &err_count, ErrorType &err, const ErrorType &err_type)
{
	if (condition)
	{
		if (err_count < ERR_COUNT_TO_REPORT)
		{
			++err_count;
			if (ERR_COUNT_TO_REPORT == err_count)
			{
				err = err_type;
			}
		}
		return true;
	}
	else
	{
		if (err_count > 0)
		{
			--err_count;
			if (0 == err_count)
			{
				err = NO_ERROR_CODE;
			}
		}
	}
	return false;
}

inline void PrintErrorCodeQueue(ErrorType *que, const int que_size)
{
    for(int i = 0; i < que_size; ++i)
    {
        if(NO_ERROR_CODE == que[i])
        {
            return;
        }
        // LOG_INFO("error code = %d", que[i]);
    }
}

} // namespace SlamCommon
#endif // !ERROR_CODE_HANDLER_H_
