#define _GNU_SOURCE
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>

static struct sigaction old_sigaction;

// 1. 自定义信号处理函数
void signal_handler(int signum, siginfo_t *siginfo, void *context) {
    if (signum == SIGSEGV) {
        size_t fault_address = (size_t)siginfo->si_addr;
        printf("\n[Signal Handler] 捕获到 SIGSEGV! 触发故障的地址: %p\n", (void*)fault_address);

        // 打印并 Sleep 60s（注意：生产环境中 sleep 和 printf 不是异步信号安全的，但在 PoC 验证中没问题）
        printf("[Signal Handler] 暂停 60 秒...\n");
        sleep(60);

        // 恢复读写权限 (PROT_READ | PROT_WRITE)
        size_t page_size = sysconf(_SC_PAGE_SIZE); // 通常是 0x1000 (4KB)
        size_t page_aligned_addr = fault_address & ~(page_size - 1);
        
        printf("[Signal Handler] 恢复页对齐地址 %p 的读写权限\n", (void*)page_aligned_addr);
        
        if (mprotect((void*)page_aligned_addr, page_size, PROT_READ | PROT_WRITE) == -1) {
            perror("[Signal Handler] mprotect 恢复权限失败");
        }

        // 返回后，CPU 会重新执行刚才触发段错误的指令
        return;
    }

    // 处理非预期的信号，透传给旧的 handler
    if (old_sigaction.sa_flags & SA_SIGINFO) {
        old_sigaction.sa_sigaction(signum, siginfo, context);
    } else if (old_sigaction.sa_handler == SIG_DFL || old_sigaction.sa_handler == SIG_IGN) {
        sigaction(signum, &old_sigaction, NULL);
    } else {
        old_sigaction.sa_handler(signum);
    }
}

// 2. 后台线程：通过命名管道接收地址输入并修改权限
void *address_listener_thread(void *arg) {
    const char *fifo_path = "/tmp/mprotect_fifo";
    
    // 创建命名管道（如果已存在则忽略错误）
    mkfifo(fifo_path, 0666);
    printf("[Listener] 监听线程已启动。请输入地址，例如在另一个终端执行: echo \"0x7ffff7a00000\" > %s\n", fifo_path);

    char buffer[256];
    while (1) {
        int fd = open(fifo_path, O_RDONLY);
        if (fd < 0) {
            sleep(1);
            continue;
        }

        ssize_t n = read(fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            
            // 将输入的十六进制字符串解析为地址
            size_t addr = strtoull(buffer, NULL, 16);
            if (addr != 0) {
                size_t page_size = sysconf(_SC_PAGE_SIZE);
                size_t aligned_addr = addr & ~(page_size - 1);
                
                printf("\n[Listener] 接收到地址输入: %p\n", (void*)addr);
                printf("[Listener] 正在将页对齐地址 %p 修改为只读...\n", (void*)aligned_addr);
                
                // 修改权限为只读（如果想连读都拦截，请改为 PROT_NONE，即 0）
                if (mprotect((void*)aligned_addr, page_size, PROT_READ) == -1) {
                    perror("[Listener] mprotect 修改权限失败");
                } else {
                    printf("[Listener] 权限修改成功！等待程序触发 SIGSEGV...\n");
                }
            }
        }
        close(fd);
    }
    return NULL;
}

// 替换入口
__attribute__((constructor))
void init() {
    printf("[LD_PRELOAD] 初始化开始...\n");

    // 替换 SIGSEGV 的信号处理机制
    struct sigaction new_sigaction;
    memset(&new_sigaction, 0, sizeof(new_sigaction));
    memset(&old_sigaction, 0, sizeof(old_sigaction));
    
    new_sigaction.sa_sigaction = signal_handler;
    new_sigaction.sa_flags = SA_RESTART | SA_SIGINFO | SA_ONSTACK;
    sigemptyset(&new_sigaction.sa_mask);

    if (sigaction(SIGSEGV, &new_sigaction, &old_sigaction) == -1) {
        perror("[LD_PRELOAD] sigaction 注册失败");
        return;
    }

    // 启动独立线程专门用于接收外部地址输入，避免阻塞主程序
    pthread_t tid;
    if (pthread_create(&tid, NULL, address_listener_thread, NULL) != 0) {
        perror("[LD_PRELOAD] 启动监听线程失败");
        return;
    }
    pthread_detach(tid); // 将线程分离，允许其在后台自动管理资源

    printf("[LD_PRELOAD] 初始化完成！\n");
}