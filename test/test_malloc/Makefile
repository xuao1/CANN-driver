# 定义编译器
CXX = g++

# 定义 Ascend 工具包路径 
ASCEND_PATH = /home/HwHiAiUser/Ascend/ascend-toolkit/latest

# 编译选项：添加头文件搜索路径
CXXFLAGS = -I$(ASCEND_PATH)/include -std=c++11 -Wall

# 链接选项：添加库文件路径和具体的库名
LDFLAGS = -L$(ASCEND_PATH)/lib64 -lascendcl

# 目标文件名
TARGET = test_malloc

# 源文件
SRCS = test_malloc.cpp

# 默认规则：编译目标程序
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(SRCS) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

# 清理规则：删除生成的可执行文件
clean:
	rm -f $(TARGET)

.PHONY: all clean