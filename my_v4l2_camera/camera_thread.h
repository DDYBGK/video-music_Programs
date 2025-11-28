/*
 * camera_thread.h
 * 摄像头控制线程的公共接口
 */

#ifndef CAMERA_THREAD_H
#define CAMERA_THREAD_H

/**
 * @brief 注册一个回调函数，当有新视频帧时，摄像头线程会调用此函数
 *
 * @param handler 函数指针, 参数为 (YUV数据指针, 数据大小)
 */
void camera_register_frame_handler(void (*handler)(void *yuv_data, int size));

/**
 * @brief 初始化V4L2, 启动摄像头捕获线程
 *
 * @param video_dev_path 摄像头设备 (e.g., "/dev/video0")
 * @param width          期望的捕获宽度 (应匹配屏幕)
 * @param height         期望的捕获高度 (应匹配屏幕)
 * @return 0 成功, -1 失败
 */
int start_camera_thread(const char *video_dev_path, int width, int height);

/*
 * 停止摄像头捕获线程并清理资源
 */
void stop_camera_thread(void);

/*
 * 调整摄像头亮度 (线程安全)
 */
void camera_change_brightness(int direction);

/**
 * 调整摄像头色度（对比度）
 */
void camera_change_contrast(int direction);

/*
 * 触发一次点击对焦 (线程安全)
 */
void camera_trigger_tap_to_focus(int x, int y);


#endif // CAMERA_THREAD_H