#pragma once
#include <stdint.h>
#include <stddef.h>

/* DMA 工具类 (SPI / 内存传输) */

#define DMA_BUFFER_ALIGN            64      /* DMA 对齐要求 (bytes) */

/* DMA 缓冲区 (自动对齐 + PSRAM 优先) */
class DMABuffer {
public:
    DMABuffer(size_t size);
    ~DMABuffer();

    void*       data()       { return _ptr; }
    const void* data() const { return _ptr; }
    size_t      size() const { return _size; }

    /* 禁止拷贝 */
    DMABuffer(const DMABuffer&) = delete;
    DMABuffer& operator=(const DMABuffer&) = delete;

private:
    void  *_ptr;
    size_t _size;
};

/* SPI DMA 批量传输辅助 */
namespace DMA {
    /* 在 PSRAM 中分配 DMA 对齐缓冲区 */
    void* allocDMABuffer(size_t size);
    void  freeDMABuffer(void *ptr);

    /* 内存 → 内存拷贝 (通过 DMA, 非阻塞), 需提前初始化 memcpy DMA 引擎 */
    // bool memcpyDMA(void *dst, const void *src, size_t len);
};
