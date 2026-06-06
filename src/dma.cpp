#include "dma.h"
#include "psram.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "DMA";

/* ================================================================ */

DMABuffer::DMABuffer(size_t size)
    : _ptr(nullptr), _size(size)
{
    /* 优先 PSRAM → 对齐分配 */
    if (PSRAM::available()) {
        _ptr = heap_caps_aligned_alloc(DMA_BUFFER_ALIGN, size,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    if (!_ptr) {
        _ptr = heap_caps_aligned_alloc(DMA_BUFFER_ALIGN, size,
                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (!_ptr) {
        ESP_LOGE(TAG, "DMA 缓冲分配失败: %d bytes", (int)size);
    }
}

DMABuffer::~DMABuffer()
{
    if (_ptr) heap_caps_free(_ptr);
}

/* ================================================================ */

void* DMA::allocDMABuffer(size_t size)
{
    void *ptr = nullptr;
    if (PSRAM::available()) {
        ptr = heap_caps_aligned_alloc(DMA_BUFFER_ALIGN, size,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    if (!ptr) {
        ptr = heap_caps_aligned_alloc(DMA_BUFFER_ALIGN, size,
                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    return ptr;
}

void DMA::freeDMABuffer(void *ptr)
{
    if (ptr) heap_caps_free(ptr);
}
