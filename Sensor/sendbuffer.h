#ifndef _SENDBUFFER_H_
#define _SENDBUFFER_H_

//#include <atomic.h>

class SendBuffer
{
public:

    SendBuffer(int capacity = 1024000);
    ~SendBuffer();

    // 向缓冲区写入数据
    bool Push(const char* data, size_t len);

    // 返回缓冲区已有数据
    char* Peek(size_t& len);

    // 弹出指定长度数据
    bool Pop(size_t len);

private:
    char* _buffer;
    size_t _capacity;
    size_t _capacity_mask;
    std::atomic_ullong _alloc_count;
    std::atomic_ullong _write_count;
    std::atomic_llong _idle_count;
    int64_t _read_count;
};

#endif
