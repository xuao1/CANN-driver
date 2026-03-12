#include <iostream>
// 引入 AscendCL 的核心头文件
#include "acl/acl.h"
#include <unistd.h> 

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

    // 3. 申请设备内存 (Device Memory)，类似于 cudaMalloc
    void* devPtr = nullptr;
    size_t size = 1024 * 1024 * 1000; // 申请 1000 MB 的显存

    std::cout << "[Info] Sleep for 30 sec" << std::endl;
    sleep(30); // 睡眠 30 秒 
    std::cout << "[Info] Attempting to allocate " << size << " bytes of device memory." << std::endl;
    sleep(3); // 睡眠 5 秒
    
    // ACL_MEM_MALLOC_HUGE_FIRST 是一种内存分配策略：优先申请大页内存，如果大页内存不足，则申请普通页内存
    ret = aclrtMalloc(&devPtr, size, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclrtMalloc failed, error code: " << ret << std::endl;
        aclrtResetDevice(deviceId);
        aclFinalize();
        return -1;
    }
    std::cout << "[Info] Successfully allocated " << size << " bytes of device memory at address: " << devPtr << std::endl;
    std::cout << "[Info] Sleep for 30 sec" << std::endl;
    sleep(30); // 睡眠 30 秒

    // 4. 释放设备内存，类似于 cudaFree
    ret = aclrtFree(devPtr);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[Error] aclrtFree failed, error code: " << ret << std::endl;
    } else {
        std::cout << "[Info] Device memory freed successfully." << std::endl;
    }

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