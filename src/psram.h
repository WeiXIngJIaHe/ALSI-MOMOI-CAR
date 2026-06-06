#pragma once
#include <stdint.h>
#include <stddef.h>

/* PSRAM 管理器 (ESP32-S3 内置 Octal PSRAM) */

class PSRAM {
public:
    /* 初始化 (需在 app_main 早期调用) */
    static bool init();

    /* 状态 */
    static bool available();
    static size_t size();        /* 总容量 (bytes) */
    static size_t freeSize();    /* 可用大小 */

    /* 分配 (从 PSRAM) */
    static void* malloc(size_t size);
    static void* calloc(size_t n, size_t size);
    static void* realloc(void *ptr, size_t size);
    static void  free(void *ptr);

    /* 信息打印 */
    static void info();

private:
    static bool _init_done;
    static bool _available;
};
