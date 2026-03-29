#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "dcmi_interface_api.h"

int main() {
    int ret = 0;
    
    ret = dcmi_init();
    if (ret != 0) {
        printf("[错误] DCMI 初始化失败，错误码: %d\n", ret);
        return ret;
    }

    int card_id = 6;      // 对应 NPU Name: 6
    int device_id = 0;    // 对应 Chip: 0
    
    unsigned int utilization_rate = 0;
    int input_type = 2;   // 2 代表查询 AI CORE

    printf("开始连续调用 1000 次 dcmi_get_device_utilization_rate...\n");

    struct timespec start, end;
    
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (int i = 0; i < 1000; i++) {
        ret = dcmi_get_device_utilization_rate(card_id, device_id, input_type, &utilization_rate);
        
        // 仅在失败时打印，避免成功的 I/O 操作污染计时结果
        if (ret != 0) {
            printf("[失败] 第 %d 次获取利用率失败，错误码: %d\n", i, ret);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    // 将 timespec 结构体的时间统一转换为微秒 (microseconds)
    long long start_usec = start.tv_sec * 1000000LL + start.tv_nsec / 1000;
    long long end_usec = end.tv_sec * 1000000LL + end.tv_nsec / 1000;
    long long elapsed_usec = end_usec - start_usec;

    printf("========================================\n");
    printf("1000 次查询总耗时: %lld 微秒 (%.3f 毫秒)\n", elapsed_usec, elapsed_usec / 1000.0);
    printf("平均每次 API 调用耗时: %.3f 微秒\n", elapsed_usec / 1000.0);
    printf("最后一次获取的 AI Core 利用率: %u%%\n", utilization_rate);
    printf("========================================\n");

    return 0;
}