#include "psram.h"
#include "esp_psram.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "PSRAM";

bool PSRAM::_init_done = false;
bool PSRAM::_available = false;

bool PSRAM::init()
{
    if (_init_done) return _available;
    _init_done = true;

    esp_err_t ret = esp_psram_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "未检测到 PSRAM, 使用内部 SRAM");
        return false;
    }

    _available = true;
    size_t sz = size();
    ESP_LOGI(TAG, "OK (%d MB)", (int)(sz / (1024 * 1024)));
    return true;
}

bool PSRAM::available()
{
    init();
    return _available;
}

size_t PSRAM::size()
{
    if (!available()) return 0;
    return esp_psram_get_size();
}

size_t PSRAM::freeSize()
{
    if (!available()) return 0;
    return heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

void* PSRAM::malloc(size_t size)
{
    if (!available()) return nullptr;
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void* PSRAM::calloc(size_t n, size_t size)
{
    if (!available()) return nullptr;
    return heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void* PSRAM::realloc(void *ptr, size_t size)
{
    if (!available()) return nullptr;
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void PSRAM::free(void *ptr)
{
    if (ptr) heap_caps_free(ptr);
}

void PSRAM::info()
{
    if (!available()) {
        ESP_LOGI(TAG, "PSRAM: 未安装");
        return;
    }
    ESP_LOGI(TAG, "PSRAM: 总量=%d KB, 可用=%d KB",
             (int)(size() / 1024), (int)(freeSize() / 1024));
}
