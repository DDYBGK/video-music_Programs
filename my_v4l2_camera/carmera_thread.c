/*
 * camera_thread.c
 * V4L2 摄像头捕获和控制逻辑的实现
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/videodev2.h>

#include "camera_thread.h"

/* --- 摄像头配置 --- */
// 分辨率现在由 init 传入
#define CAMERA_FORMAT V4L2_PIX_FMT_YUV420 // 您指定的 YUV420 格式
#define NB_BUFFER     4

/* --- 内部全局 (静态) 变量 --- */
static int g_camera_fd = -1;
static unsigned char *g_pucVideBuf[NB_BUFFER];
static int g_iVideoBufLen[NB_BUFFER];
static int g_num_buffers = 0; 

static pthread_t g_capture_thread_id;
static volatile int g_camera_running = 0; 

// 回调函数指针
static void (*g_frame_handler)(void *data, int size) = NULL;

// 控件相关
static pthread_mutex_t g_v4l2_lock; 
static struct v4l2_queryctrl g_brightness_qctrl;
static int g_brightness_delta = 0;
static struct v4l2_queryctrl g_contrast_qctrl; // 对比度查询结构
static int g_contrast_delta = 0;               // 对比度步进值

static time_t g_tap_focus_timestamp = 0; 
#define AF_TIMEOUT_SEC 5                 

/*
 * ===================================================================
 * V4L2 控件辅助函数 (内部)
 * ===================================================================
 */

static int v4l2_query_ctrl(int fd, int id, struct v4l2_queryctrl *qctrl) {
    qctrl->id = id;  //id号
    if (0 == ioctl(fd, VIDIOC_QUERYCTRL, qctrl)) {
        printf("[CAM] 查询控件(ID: 0x%x)成功: min=%d, max=%d, step=%d\n",
               id, qctrl->minimum, qctrl->maximum, qctrl->step);
        return 0;
    }
    return -1;
}

static int v4l2_set_control(int fd, int id, int value) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    ctrl.value = value;
    pthread_mutex_lock(&g_v4l2_lock);
    int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);  //设置了控制件的值
    pthread_mutex_unlock(&g_v4l2_lock);
    return ret;
}

static int v4l2_get_control(int fd, int id, int *value) {
    struct v4l2_control ctrl;
    ctrl.id = id;
    pthread_mutex_lock(&g_v4l2_lock);
    if (0 != ioctl(fd, VIDIOC_G_CTRL, &ctrl)) {
        perror("[CAM] ioctl (VIDIOC_G_CTRL)");
        pthread_mutex_unlock(&g_v4l2_lock);
        return -1;
    }
    pthread_mutex_unlock(&g_v4l2_lock);
    *value = ctrl.value;
    return 0;
}

/* 开启连续自动对焦 (内部) */
static void enable_continuous_af(void) {
    printf("[CAM] 开启连续自动对焦 (Continuous AF)\n");
    v4l2_set_control(g_camera_fd, V4L2_CID_FOCUS_AUTO, 1);
    g_tap_focus_timestamp = 0; 
}


/*
 * ===================================================================
 * 摄像头捕获线程 (内部)
 * ===================================================================
 */
static void* camera_capture_loop(void* args) {
    printf("[CAM] 捕获线程已启动...\n");

    while (g_camera_running) {
        /* 1. poll 监测数据 */
        struct pollfd pfd;
        pfd.fd = g_camera_fd;
        pfd.events = POLLIN;
        int ret = poll(&pfd, 1, 200); // 200ms 超时

        if (ret < 0) {
            if (!g_camera_running) break; //如果此时关闭就跳出了
            perror("[CAM] poll error");
            break;
        } else if (ret == 0) {
            // Poll 超时, 检查 AF
            if (g_tap_focus_timestamp != 0 && (time(NULL) - g_tap_focus_timestamp > AF_TIMEOUT_SEC)) {
                printf("[CAM] 点击对焦超时，恢复连续 AF...\n");
                enable_continuous_af();
            }
            continue; 
        }

        /* 2. 从队列取出 (DQBUF) */
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (0 != ioctl(g_camera_fd, VIDIOC_DQBUF, &buf)) {
            perror("[CAM] VIDIOC_DQBUF");
            break;
        }

        /* * 3. 处理数据 (调用回调) 
         * 不再保存文件，而是将缓冲区指针和大小传递给主线程
         */
        if (g_frame_handler) {
            //回调函数的handler()没有具体的实现等一下自己实现
            //g_pucVideBuf[buf.index]是内存映射的地址，这里就可以直接访问地址了
            g_frame_handler(g_pucVideBuf[buf.index], buf.bytesused);  
        }

        /* 4. 将 buffer 重新放回队列 (QBUF) */
        if (0 != ioctl(g_camera_fd, VIDIOC_QBUF, &buf)) {
            perror("[CAM] VIDIOC_QBUF");
            break;
        }
    }
    
    printf("[CAM] 捕获线程已退出。\n");
    return NULL;
}


/*
 * ===================================================================
 * 摄像头初始化 (内部)
 * ===================================================================
 */
static int init_camera(const char *dev_name, int width, int height) {
    int fd = -1;
    struct v4l2_capability capability;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req_bufs;
    int i;

    /* 1. 打开设备 */
    fd = open(dev_name, O_RDWR);
    if (fd < 0) {
        perror("[CAM] 打开摄像头失败");
        return -1;
    }

    /* 2. 查询能力 */
    if (0 != ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        perror("[CAM] VIDIOC_QUERYCAP");
        goto err_exit;
    }
    if (!(capability.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "[CAM] 设备不支持 streaming (mmap)\n");
        goto err_exit;
    }
    printf("[CAM] 摄像头驱动: %s, 卡: %s\n", capability.driver, capability.card);

    /* 3. 设置格式 (使用传入的分辨率) */
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = CAMERA_FORMAT; // V4L2_PIX_FMT_YUV420
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (0 != ioctl(fd, VIDIOC_S_FMT, &fmt)) {
        perror("[CAM] VIDIOC_S_FMT 设置格式失败");
        fprintf(stderr, "[CAM] 您的摄像头可能不支持 %dx%d YUV420 格式\n", width, height);
        goto err_exit;
    }
    printf("[CAM] 设置格式成功: %dx%d, 格式: YUV420\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
    // 检查驱动是否修改了分辨率
    if(fmt.fmt.pix.width != width || fmt.fmt.pix.height != height) {
        printf("[CAM] 警告: 驱动将分辨率调整为 %dx%d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
    }

    /* 4. 请求缓冲区 */
    memset(&req_bufs, 0, sizeof(struct v4l2_requestbuffers));
    req_bufs.count = NB_BUFFER;
    req_bufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req_bufs.memory = V4L2_MEMORY_MMAP;
    if (0 != ioctl(fd, VIDIOC_REQBUFS, &req_bufs)) {
        perror("[CAM] VIDIOC_REQBUFS");
        goto err_exit;
    }
    g_num_buffers = req_bufs.count; 
    printf("[CAM] 实际分配到 %d 个缓冲区\n", g_num_buffers);

    /* 5. 映射 (mmap) 缓冲区 */
    for (i = 0; i < g_num_buffers; i++) {
        struct v4l2_buffer vb;
        memset(&vb, 0, sizeof(struct v4l2_buffer));
        vb.index = i;
        vb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vb.memory = V4L2_MEMORY_MMAP;

        if (0 != ioctl(fd, VIDIOC_QUERYBUF, &vb)) {
            perror("[CAM] VIDIOC_QUERYBUF");
            goto err_exit; 
        }

        g_iVideoBufLen[i] = vb.length;
        g_pucVideBuf[i] = (unsigned char *)mmap(0, vb.length,
                                                PROT_READ | PROT_WRITE,
                                                MAP_SHARED,
                                                fd, vb.m.offset);
        if (g_pucVideBuf[i] == MAP_FAILED) {
            perror("[CAM] mmap 失败");
            goto err_exit;
        }
    }

    /* 6. 将缓冲区放入队列 (QBUF) */
    for (i = 0; i < g_num_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (0 != ioctl(fd, VIDIOC_QBUF, &buf)) {
            perror("[CAM] VIDIOC_QBUF");
            goto err_exit;
        }
    }

    /* 7. 初始化控件 */
    if (0 == v4l2_query_ctrl(fd, V4L2_CID_BRIGHTNESS, &g_brightness_qctrl)) {
        g_brightness_delta = (g_brightness_qctrl.maximum - g_brightness_qctrl.minimum) / 20;
        if (g_brightness_delta == 0) g_brightness_delta = g_brightness_qctrl.step;
    } else {
        printf("[CAM] 警告: 驱动不支持亮度(Brightness)调节\n");
    }
    if (0 == v4l2_query_ctrl(fd, V4L2_CID_CONTRAST, &g_contrast_qctrl)) {
        g_contrast_delta = (g_contrast_qctrl.maximum - g_contrast_qctrl.minimum) / 20;
        if (g_contrast_delta == 0) g_contrast_delta = g_contrast_qctrl.step;
    } else {
        printf("[CAM] 警告: 驱动不支持对比度(Contrast)调节\n");
    }
    struct v4l2_queryctrl af_qctrl_temp; 
    v4l2_query_ctrl(fd, V4L2_CID_FOCUS_AUTO, &af_qctrl_temp); // 仅查询
    
    return fd;

err_exit:
    close(fd);
    return -1;
}

/*
 * ===================================================================
 * 公共接口函数实现
 * ===================================================================
 */

/* 注册回调 */
void camera_register_frame_handler(void (*handler)(void *yuv_data, int size)) {
    g_frame_handler = handler;
}

/* 调整亮度 (外部调用) */
void camera_change_brightness(int direction) {
    if (g_camera_fd < 0 || g_brightness_delta == 0) return;
    int current_val;
    if (0 != v4l2_get_control(g_camera_fd, V4L2_CID_BRIGHTNESS, &current_val)) {
        return;
    }
    current_val += (direction > 0) ? g_brightness_delta : -g_brightness_delta;
    if (current_val > g_brightness_qctrl.maximum) current_val = g_brightness_qctrl.maximum;
    if (current_val < g_brightness_qctrl.minimum) current_val = g_brightness_qctrl.minimum;
    printf("[CAM] 设置亮度为: %d\n", current_val);
    
    v4l2_set_control(g_camera_fd, V4L2_CID_BRIGHTNESS, current_val);
}

/* 调整对比度（外部调用），水平滑动来实现设置 */
void camera_change_contrast(int direction){
    if(g_camera_fd<0 || g_contrast_delta==0) return;
    int current_val;
    //获取对比度
    if(0 != v4l2_get_control(g_camera_fd, V4L2_CID_CONTRAST, &current_val)){
        return ;
    }
    current_val += (direction > 0) ? g_contrast_delta : -g_contrast_delta;
    if(current_val > g_contrast_qctrl.maximum) current_val = g_contrast_qctrl.maximum;
    if(current_val < g_contrast_qctrl.minimum) current_val = g_contrast_qctrl.minimum;
    printf("[CAM] 设置对比度为 %d\n", current_val);
    v4l2_set_control(g_camera_fd, V4L2_CID_CONTRAST, current_val);
}

/* 触发点击对焦 (外部调用) */
void camera_trigger_tap_to_focus(int x, int y) {
    if (g_camera_fd < 0) return;
    printf("[CAM] 在 (%d, %d) 触发点击对焦\n", x, y);
    v4l2_set_control(g_camera_fd, V4L2_CID_FOCUS_AUTO, 0);
    // (TODO: 在此设置 V4L2_CID_AUTO_FOCUS_ROI (如果驱动支持))
    //获取当前的格式以确定图像尺寸
    struct v4l2_queryctrl qctrl;
    if(0 == v4l2queryctrl(g_camera_fd, V4L2_CID_AUTO_FOCUS_ROI, &qctrl)){
        struct v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if(0 == ioctl(g_camera_fd, VIDIOC_G_FMT, &fmt)){
            int width = fmt.fmt.pix.width;
            int height = fmt.fmt.height;
            //设置对焦区域
            struct v4l2_ext_controls ext_ctrls;
            struct v4l2_ext_control ext_ctrl;
            struct v4l2_area focus_area;

            // 设置对焦区域 (以点击点为中心的100x100区域)
            focus_area.width = 100;
            focus_area.height = 100;
            focus_area.left = (x > 50) ? (x - 50) : 0;
            focus_area.top = (y > 50) ? (y - 50) : 0;

            // 确保区域不超出边界
            if (focus_area.left + focus_area.width > width) {
                focus_area.left = width - focus_area.width;
            }
            if (focus_area.top + focus_area.height > height) {
                focus_area.top = height - focus_area.height;
            }
            //配置扩展控件
            ext_ctrl.id = V4L2_CID_AUTO_FOCUS_ROI;
            ext_ctrl.size = sizeof(focus_area);
            ext_ctrl.p_area = &focus_area;

            ext_ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
            ext_ctrls.count = 1;
            ext_ctrls.controls = &ext_ctrl;

            pthread_mutex_lock(&g_v4l2_lock);
            int ret = ioctl(g_camera_fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls);
            pthread_mutex_unlock(&g_v4l2_lock);
            if(ret < 0){
                printf("[CAM] 设置对焦区域失败: %s\n", strerror(errno));
            }else{
                 printf("[CAM] 成功设置对焦区域: (%d,%d) %dx%d\n", 
                       focus_area.left, focus_area.top, focus_area.width, focus_area.height);
            }
        }
    }else{
        printf("[CAM] 驱动不支持对焦区域控制\n");
    }

    v4l2_set_control(g_camera_fd, V4L2_CID_AUTO_FOCUS_START, 1);
    g_tap_focus_timestamp = time(NULL);
}


/* 启动摄像头 (外部调用) */
int start_camera_thread(const char *video_dev_path, int width, int height) {
    int type;
    pthread_mutex_init(&g_v4l2_lock, NULL);

    g_camera_fd = init_camera(video_dev_path, width, height);
    if (g_camera_fd < 0) {
        return -1;
    }

    /* 启动流 */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 != ioctl(g_camera_fd, VIDIOC_STREAMON, &type)) {
        perror("[CAM] VIDIOC_STREAMON");
        close(g_camera_fd);
        return -1;
    }
    printf("[CAM] 摄像头启动成功! (STREAMON)\n");

    /* 设置初始状态: 连续自动对焦 */
    enable_continuous_af();

    /* 启动捕获线程 */
    g_camera_running = 1;
    if (0 != pthread_create(&g_capture_thread_id, NULL, camera_capture_loop, NULL)) {
        perror("[CAM] 无法创建捕获线程");
        g_camera_running = 0;
        ioctl(g_camera_fd, VIDIOC_STREAMOFF, &type);
        close(g_camera_fd);
        return -1;
    }
    return 0; // 成功
}

/* 停止摄像头 (外部调用) */为0是什么意思
void stop_camera_thread(void) {
    int i;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!g_camera_running) return;
    
    printf("[CAM] 正在停止摄像头线程...\n");
    g_camera_running = 0;
    pthread_join(g_capture_thread_id, NULL);

    if (g_camera_fd >= 0) {
        ioctl(g_camera_fd, VIDIOC_STREAMOFF, &type);
        for (i = 0; i < g_num_buffers; i++) {
            if (g_pucVideBuf[i] != MAP_FAILED) {
                munmap(g_pucVideBuf[i], g_iVideoBufLen[i]);
            }
        }
        close(g_camera_fd);
        g_camera_fd = -1;
    }
    pthread_mutex_destroy(&g_v4l2_lock);
    printf("[CAM] 摄像头已停止。\n");
}