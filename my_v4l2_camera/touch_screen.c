/*
 * touch_display.c
 * 主程序: 处理触摸屏输入 (/dev/input/eventY)
 * 处理显示输出 (/dev/fb0)
 * 协调摄像头线程
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h> 
#include <string.h>
#include <sys/mman.h>
#include <linux/fb.h> // Framebuffer
#include <linux/input.h>
#include <libevdev/libevdev.h> 

#include "camera_thread.h" // 包含我们的相机控制接口

/* --- 全局变量 --- */

// Framebuffer
static int g_fb_fd = -1;
static char *g_fb_mem = NULL;                 // mmap 的显存指针
static struct fb_var_screeninfo g_vinfo;      // 屏幕可变信息
static struct fb_fix_screeninfo g_finfo;      // 屏幕固定信息
static long g_screen_mem_size = 0;            // 显存总大小
static int g_bytes_per_pixel = 0;
static unsigned char *g_rgb_buffer = NULL;    // 用于 YUV -> RGB 转换的中间缓冲区

// Touch
struct libevdev *g_touch_dev = NULL;  //用于Linux input子系统
static int g_touch_fd = -1;

// 状态
static volatile int g_main_running = 1;

// 触摸状态机
struct touch_state {
    int start_x, start_y, end_x, end_y;
    int is_touching;
    long start_time_ms;
};

/* --- Ctrl+C 信号处理 --- */
void sigint_handler(int sig) {
    printf("\n[MAIN] 收到 Ctrl+C (SIGINT), 正在退出...\n");
    g_main_running = 0;
}

static long get_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}


/*
 * ===================================================================
 * 色彩空间转换 (YUV420 Planar -> RGB)
 * ===================================================================
 */

// 辅助函数: 将值限制在 0-255
static inline unsigned char clamp(int value) {
    if (value < 0) return 0;
    if (value > 255) return 255;
    return (unsigned char)value;
}

/**
 * @brief YUV420P (I420) 转换为 RGB565 (16-bit)
 * YUV420P 格式: W*H 个 Y, (W/2)*(H/2) 个 U, (W/2)*(H/2) 个 V
 */
void yuv420p_to_rgb565(unsigned char *pY, unsigned char *pU, unsigned char *pV, 
                         unsigned short *rgb_output, int width, int height) 
{
    int y, x;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            // 获取 Y
            int Y = pY[y * width + x];
            // 获取 U, V (每 2x2 共享一组 UV)
            int U = pU[(y / 2) * (width / 2) + (x / 2)];
            int V = pV[(y / 2) * (width / 2) + (x / 2)];

            // YUV -> RGB (标准公式, Cb=U, Cr=V)
            Y -= 16;
            U -= 128;
            V -= 128;
            int R = clamp((298 * Y + 409 * V + 128) >> 8);
            int G = clamp((298 * Y - 100 * U - 208 * V + 128) >> 8);
            int B = clamp((298 * Y + 516 * U + 128) >> 8);

            // RGB888 -> RGB565 (R 5bit, G 6bit, B 5bit)
            rgb_output[y * width + x] = ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | ((B & 0xF8) >> 3);
        }
    }
}

/**
 * @brief YUV420P (I420) 转换为 RGB8888 (32-bit, A-R-G-B 或 X-R-G-B)
 * 这里假设高位字节是 Alpha 或不用 (X)
 */
void yuv420p_to_rgb8888(unsigned char *pY, unsigned char *pU, unsigned char *pV, 
                          unsigned int *rgb_output, int width, int height) 
{
    int y, x;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int Y = pY[y * width + x];
            int U = pU[(y / 2) * (width / 2) + (x / 2)];
            int V = pV[(y / 2) * (width / 2) + (x / 2)];

            Y -= 16;
            U -= 128;
            V -= 128;
            int R = clamp((298 * Y + 409 * V + 128) >> 8);
            int G = clamp((298 * Y - 100 * U - 208 * V + 128) >> 8);
            int B = clamp((298 * Y + 516 * U + 128) >> 8);

            // 存为 0xAARRGGBB 格式 (Alpha=255)
            rgb_output[y * width + x] = (0xFF << 24) | (R << 16) | (G << 8) | B;
            
            // 注意: Framebuffer 可能是 BGRA 格式, 此时应为:
            // (0xFF << 24) | (B << 16) | (G << 8) | R;
            // (请根据 g_vinfo.red, g_vinfo.green, g_vinfo.blue 的 offset 成员判断)
        }
    }
}


/*
 * ===================================================================
 * 摄像头帧处理回调
 * ===================================================================
 */

/**
 * @brief 这是被 camera_thread 调用的回调函数
 * 它在 camera_thread 的上下文中执行！
 */
void process_frame(void *yuv_data, int size) {
    if (!g_rgb_buffer || !g_fb_mem) return;

    int width = g_vinfo.xres;
    int height = g_vinfo.yres;

    // YUV420P (I420) 内存布局
    unsigned char *pY = (unsigned char *)yuv_data;
    unsigned char *pU = pY + (width * height);
    unsigned char *pV = pU + (width * height / 4);

    // 1. 色彩转换
    if (g_bytes_per_pixel == 2) {
        // 16-bit (RGB565)
        yuv420p_to_rgb565(pY, pU, pV, (unsigned short*)g_rgb_buffer, width, height);
    } else if (g_bytes_per_pixel == 4) {
        // 32-bit (RGB8888)
        yuv420p_to_rgb8888(pY, pU, pV, (unsigned int*)g_rgb_buffer, width, height);
    } else {
        // 不支持的 BPP
        return;
    }

    // 2. 将 RGB 数据刷到 Framebuffer
    // 使用 g_finfo.line_length (行长) 而不是 W*BPP，以处理屏幕 padding
    int y;
    for(y = 0; y < height; y++) {
        memcpy(g_fb_mem + y * g_finfo.line_length,     // 目标: 显存的第 y 行
               g_rgb_buffer + y * width * g_bytes_per_pixel, // 源: RGB 缓冲区的第 y 行
               width * g_bytes_per_pixel);                 // 拷贝长度
    }
    
    // (注意: 真正的应用会使用双缓冲和 VIDIOC_PAN_DISPLAY 来避免闪烁)
}


/*
 * ===================================================================
 * 初始化/清理
 * ===================================================================
 */

int init_framebuffer(const char *fb_dev) {
    g_fb_fd = open(fb_dev, O_RDWR);
    if (g_fb_fd < 0) {
        perror("[MAIN] 无法打开 Framebuffer 设备");
        return -1;
    }

    // 获取屏幕可变信息 (分辨率, BPP)
    if (0 > ioctl(g_fb_fd, FBIOGET_VSCREENINFO, &g_vinfo)) {
        perror("[MAIN] 无法获取 VSCREENINFO");
        close(g_fb_fd);
        return -1;
    }
    
    // 获取屏幕固定信息 (行长)
    if (0 > ioctl(g_fb_fd, FBIOGET_FSCREENINFO, &g_finfo)) {
        perror("[MAIN] 无法获取 FSCREENINFO");
        close(g_fb_fd);
        return -1;
    }

    g_bytes_per_pixel = g_vinfo.bits_per_pixel / 8;
    g_screen_mem_size = g_vinfo.yres * g_finfo.line_length; // 使用行长计算总大小

    printf("[MAIN] 显示屏信息: %dx%d, %d bpp (BPP=%d), 行长: %d\n",
           g_vinfo.xres, g_vinfo.yres, g_vinfo.bits_per_pixel, g_bytes_per_pixel, g_finfo.line_length);
    
    if (g_bytes_per_pixel != 2 && g_bytes_per_pixel != 4) {
        fprintf(stderr, "[MAIN] 错误: 不支持 %d bpp, 仅支持 16 或 32 bpp\n", g_vinfo.bits_per_pixel);
        close(g_fb_fd);
        return -1;
    }

    // mmap 显存
    g_fb_mem = (char *)mmap(NULL, g_screen_mem_size,
                            PROT_READ | PROT_WRITE, MAP_SHARED,
                            g_fb_fd, 0);
    if (g_fb_mem == MAP_FAILED) {
        perror("[MAIN] mmap 显存失败");
        close(g_fb_fd);
        return -1;
    }
    
    // 分配 RGB 中间缓冲区
    g_rgb_buffer = (unsigned char *)malloc(g_vinfo.xres * g_vinfo.yres * g_bytes_per_pixel);
    if (!g_rgb_buffer) {
        fprintf(stderr, "[MAIN] 分配 RGB 缓冲区失败\n");
        munmap(g_fb_mem, g_screen_mem_size);
        close(g_fb_fd);
        return -1;
    }

    // 清屏 (黑色)
    memset(g_fb_mem, 0, g_screen_mem_size);
    return 0; // 成功
}

int init_touch(const char *touch_dev) {
    g_touch_fd = open(touch_dev, O_RDONLY | O_NONBLOCK);
    if (g_touch_fd < 0) {
        perror("[MAIN] 无法打开触摸设备");
        return -1;
    }
    int rc = libevdev_new_from_fd(g_touch_fd, &g_touch_dev);
    if (rc < 0) {
        fprintf(stderr, "[MAIN] 无法初始化 libevdev: %s\n", strerror(-rc));
        close(g_touch_fd);
        return -1;
    }
    printf("[MAIN] 触摸设备: %s\n", libevdev_get_name(g_touch_dev));
    return 0; // 成功
}

void cleanup_all() {
    if (g_rgb_buffer) {
        free(g_rgb_buffer);
        g_rgb_buffer = NULL;
    }
    if (g_fb_mem != MAP_FAILED && g_fb_mem != NULL) {
        munmap(g_fb_mem, g_screen_mem_size);
        g_fb_mem = NULL;
    }
    if (g_fb_fd >= 0) {
        close(g_fb_fd);
        g_fb_fd = -1;
    }
    if (g_touch_dev) {
        libevdev_free(g_touch_dev);
        g_touch_dev = NULL;
    }
    if (g_touch_fd >= 0) {
        close(g_touch_fd);
        g_touch_fd = -1;
    }
}


/*
 * ===================================================================
 * 主函数 (Main)
 * ===================================================================
 */
int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("用法: %s </dev/videoX> </dev/input/eventY> </dev/fbZ>\n", argv[0]);
        printf("示例: %s /dev/video0 /dev/input/event1 /dev/fb0\n", argv[0]);
        return -1;
    }
    
    char *video_dev = argv[1];
    char *touch_dev = argv[2];
    char *fb_dev = argv[3];
    
    /* 1. 设置 Ctrl+C 信号处理器 */
    signal(SIGINT, sigint_handler);

    /* 2. 初始化显示屏 (必须在摄像头之前, 以获取分辨率) */
    if (0 != init_framebuffer(fb_dev)) {
        return -1;
    }

    /* 3. 初始化触摸屏 */
    if (0 != init_touch(touch_dev)) {
        cleanup_all();
        return -1;
    }

    /* 4. 注册摄像头回调 */
    camera_register_frame_handler(process_frame);

    /* 5. 启动摄像头线程 (传入屏幕分辨率) */
    printf("[MAIN] 正在启动摄像头线程...\n");
    if (0 != start_camera_thread(video_dev, g_vinfo.xres, g_vinfo.yres)) {
        fprintf(stderr, "[MAIN] 启动摄像头失败!\n");
        cleanup_all();
        return -1;
    }
    printf("[MAIN] 摄像头线程启动成功。\n");
    printf("[MAIN] 初始化完成。等待触摸输入... (按 Ctrl+C 退出)\n");

    /* 6. 主循环 (触摸事件处理) */
    struct touch_state state = {0};
    const int SWIPE_THRESHOLD_Y = 100;
    const int TAP_TIME_MS = 200;

    while (g_main_running) {
        struct input_event ev;
        int rc = libevdev_next_event(g_touch_dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        
        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            
            if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
                if (ev.value == 1) { // 手指按下
                    state.is_touching = 1;
                    state.start_time_ms = get_time_ms();
                } else { // 手指抬起
                    state.is_touching = 0;
                    long duration = get_time_ms() - state.start_time_ms;
                    int delta_y = state.end_y - state.start_y;

                    if (abs(delta_y) > SWIPE_THRESHOLD_Y) {
                        if (delta_y < 0) {
                            printf("[MAIN] 手势: 上划 (亮度 +)\n");
                            camera_change_brightness(1); // 调用相机接口
                        } else {
                            printf("[MAIN] 手势: 下滑 (亮度 -)\n");
                            camera_change_brightness(-1); // 调用相机接口
                        }
                    }else if (abs(delta_x) > SWIPE_THRESHOLD_X) {
                        if (delta_x > 0) {
                            printf("[MAIN] 手势: 右划 (对比度 +)\n");
                            camera_change_contrast(1); // 调用对比度接口
                        } else {
                            printf("[MAIN] 手势: 左划 (对比度 -)\n");
                            camera_change_contrast(-1); // 调用对比度接口
                        }
                    }else if (duration < TAP_TIME_MS) {
                        printf("[MAIN] 手势: 点击 @ (%d, %d)\n", state.end_x, state.end_y);
                        camera_trigger_tap_to_focus(state.end_x, state.end_y); // 调用相机接口
                    }
                    memset(&state, 0, sizeof(state));
                }
            }
            
            if (state.is_touching && ev.type == EV_ABS) {
                if (ev.code == ABS_X) {
                    if (state.start_x == 0) state.start_x = ev.value;
                    state.end_x = ev.value;
                } else if (ev.code == ABS_Y) {
                    if (state.start_y == 0) state.start_y = ev.value;
                    state.end_y = ev.value;
                }
            }

        } else if (rc == -EAGAIN) {
            usleep(20 * 1000); // 20ms
        } else if (rc < 0) {
            perror("[MAIN] libevdev_next_event");
            g_main_running = 0; 
        }
    } // end while(g_main_running)

    /* 7. 清理 */
    printf("[MAIN] 正在清理资源...\n");
    stop_camera_thread(); // 通知相机线程停止
    cleanup_all();        // 清理 FB 和 Touch

    printf("[MAIN] 程序退出。\n");
    return 0;
}