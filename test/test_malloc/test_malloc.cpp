#include <iostream>
#include <vector>
#include <string>
// 引入 AscendCL 的核心头文件
#include "acl/acl.h"
#include <unistd.h> 

// 辅助函数：将字节数格式化为易读的字符串，方便看日志
std::string formatSize(size_t size) {
    if (size >= 1024ULL * 1024 * 1024) return std::to_string(size / (1024ULL * 1024 * 1024)) + " GiB";
    if (size >= 1024 * 1024) return std::to_string(size / (1024 * 1024)) + " MiB";
    if (size >= 1024) return std::to_string(size / 1024) + " KiB";
    return std::to_string(size) + " B";
}

int main() {
    // 1. 初始化 ACL (参数为空指针即可)
    aclError ret = aclInit(nullptr);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclInit failed, error code: " << ret << std::endl;
        return -1;
    }
    std::cout << "[Info] ACL initialized successfully." << std::endl;

    // 2. 指定运行的设备 (比如使用卡 0)
    int32_t deviceId = 0;
    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclrtSetDevice failed, error code: " << ret << std::endl;
        aclFinalize();
        return -1;
    }
    std::cout << "[Info] Device " << deviceId << " set successfully." << std::endl;

    // 定义需要测试的内存大小列表 (单位: Bytes)
    // 必须使用 ULL 防止大容量计算时发生整型溢出
    std::vector<size_t> testSizes = {
        // 1024ULL,                                // 1 KiB
        1024ULL * 1024,                         // 1 MiB
        2ULL * 1024 * 1024,                     // 2 MiB
        64ULL * 1024 * 1024,                    // 64 MiB
        1024ULL * 1024 * 1024,                  // 1 GiB
        2ULL * 1024 * 1024 * 1024,              // 2 GiB
        16ULL * 1024 * 1024 * 1024,             // 16 GiB
        30ULL * 1024 * 1024 * 1024,             // 30 GiB
        36ULL * 1024 * 1024 * 1024              // 36 GiB
    };

    // 3 & 4. 循环进行：申请 -> 休眠 -> 释放
    for (size_t size : testSizes) {
        void* devPtr = nullptr;
        
        std::cout << "\n==================================================" << std::endl;
        std::cout << "[Info] Target allocation size: " << formatSize(size) << " (" << size << " bytes)" << std::endl;
        
        // 申请前休眠 30 秒
        std::cout << "[Info] Sleep for 30 sec before allocation..." << std::endl;
        sleep(30); 
        
        std::cout << "[Info] Attempting to allocate " << formatSize(size) << " of device memory." << std::endl;
        
        // 申请设备内存
        ret = aclrtMalloc(&devPtr, size, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            std::cerr << "[Error] aclrtMalloc failed for size " << formatSize(size) 
                      << ", error code: " << ret << std::endl;
            // 一旦申请失败（比如 36GiB 超出显存上限），清理环境并退出
            aclrtResetDevice(deviceId);
            aclFinalize();
            return -1;
        }
        std::cout << "[Info] Successfully allocated at address: " << devPtr << std::endl;
        
        // 申请后休眠 30 秒
        std::cout << "[Info] Sleep for 30 sec after allocation..." << std::endl;
        sleep(30); 

        // 释放设备内存
        ret = aclrtFree(devPtr);
        if (ret != ACL_SUCCESS) {
            std::cerr << "[Error] aclrtFree failed, error code: " << ret << std::endl;
            aclrtResetDevice(deviceId);
            aclFinalize();
            return -1;
        } else {
            std::cout << "[Info] Device memory freed successfully." << std::endl;
        }
    }

    std::cout << "\n==================================================" << std::endl;
    std::cout << "[Info] All memory tests completed." << std::endl;

    // 5. 释放设备
    ret = aclrtResetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclrtResetDevice failed, error code: " << ret << std::endl;
    }

    // 6. 去初始化 ACL
    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclFinalize failed, error code: " << ret << std::endl;
    }
    std::cout << "[Info] ACL finalized successfully." << std::endl;

    return 0;
}