#include <stdio.h>
#include <unistd.h>
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

    printf("开始轮询 NPU %d Chip %d 的 AI Core 利用率 (Ctrl+C 退出)...\n", card_id, device_id);

    while (1) {
        ret = dcmi_get_device_utilization_rate(card_id, device_id, input_type, &utilization_rate);
        
        if (ret == 0) {
            printf("[成功] 当前 AI Core 利用率: %u%%\n", utilization_rate);
        } else {
            printf("[失败] 获取利用率失败，错误码: %d\n", ret);
        }
        
        sleep(1); 
    }

    return 0;
}