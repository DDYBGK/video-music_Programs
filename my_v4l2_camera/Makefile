# Makefile for Camera Touch Display App

# 交叉编译器前缀 (如果需要，请修改)
#TARGET_PREFIX = aarch64-linux-gnu-
CC = $(TARGET_PREFIX)gcc

# 编译和链接标志
CFLAGS = -Wall -Wextra -O2 -I.
# LDFLAGS: -pthread (线程) -levdev (触摸) -lv4l2 (摄像头)
LDFLAGS = -lpthread -levdev -lv4l2 -lm (数学库, 用于色彩转换)

# 源文件
SRCS = touch_screen.c camera_thread.c
# 目标程序
TARGET = camera_touch_app

all: $(TARGET)

$(TARGET): $(SRCS) camera_thread.h
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)