#include "rkmedia_assignment_manage.h"
#include "rkmedia_data_process.h"
#include "rkmedia_ffmpeg_config.h"
#include "rkmedia_module.h"

#include "rkmedia_ffmpeg_config.h"
#include "rkmedia_container.h"

int init_rv1126_first_assignment(int protocol_type, char * network_address, int low_url_type, char *low_url_address)
{
    int ret;
    RKMEDIA_FFMPEG_CONFIG *ffmpeg_config = (RKMEDIA_FFMPEG_CONFIG *)malloc(sizeof(RKMEDIA_FFMPEG_CONFIG));
    if (ffmpeg_config == NULL)
    {
        printf("malloc ffmpeg_config failed\n");
    }
    ffmpeg_config->width = 1920;
    ffmpeg_config->height = 1080;
    ffmpeg_config->config_id = 0;
    ffmpeg_config->protocol_type = protocol_type;
    ffmpeg_config->video_codec = AV_CODEC_ID_H264;
    ffmpeg_config->audio_codec = AV_CODEC_ID_AAC;
    memcpy(ffmpeg_config->network_addr, network_address, strlen(network_address));
    //初始化ffmpeg输出模块
    init_rkmedia_ffmpeg_context(ffmpeg_config);

    RKMEDIA_FFMPEG_CONFIG *low_ffmpeg_config = (RKMEDIA_FFMPEG_CONFIG *)malloc(sizeof(RKMEDIA_FFMPEG_CONFIG));
    if (ffmpeg_config == NULL)
    {
        printf("malloc ffmpeg_config failed\n");
    }

    //char  * low_network_address = "rtmp://192.168.1.66:1935/live/02";
    low_ffmpeg_config->width = 1280;
    low_ffmpeg_config->height = 720;
    low_ffmpeg_config->config_id = 1;
    low_ffmpeg_config->protocol_type = protocol_type;
    low_ffmpeg_config->video_codec = AV_CODEC_ID_H264;
    low_ffmpeg_config->audio_codec = AV_CODEC_ID_AAC;
    //memcpy(low_ffmpeg_config->network_addr, low_network_address, strlen(low_network_address));
    memcpy(low_ffmpeg_config->network_addr, low_url_address, strlen(low_url_address));

    init_rkmedia_ffmpeg_context(low_ffmpeg_config);

    printf("Bind Before...\n");


    /*********************以上是对FFmpeg的设定，下面是RV1126的操作*************************/
    MPP_CHN_S vi_channel;
    MPP_CHN_S venc_channel;

    MPP_CHN_S rga_channel;
    MPP_CHN_S low_venc_channel;
    
    //从VI容器里面获取VI_ID
    RV1126_VI_CONTAINTER vi_container;
    get_vi_container(0, &vi_container);

    //从VENC容器里面获取VENC_ID
    RV1126_VENC_CONTAINER venc_container;
    get_venc_container(0, &venc_container);

    vi_channel.enModId = RK_ID_VI;  //VI模块ID
    vi_channel.s32ChnId = vi_container.vi_id;//VI通道ID
    
    venc_channel.enModId = RK_ID_VENC;//VENC模块ID
    venc_channel.s32ChnId = venc_container.venc_id;//VENC通道ID

    //绑定VI和VENC节点
    ret = RK_MPI_SYS_Bind(&vi_channel, &venc_channel);
    if (ret != 0)
    {
        printf("bind venc error\n");
        return -1;
    }
    else
    {
        printf("bind venc success\n");
    }

    RV1126_VENC_CONTAINER low_venc_container;
    get_venc_container(1, &low_venc_container);
    rga_channel.enModId = RK_ID_RGA;
    rga_channel.s32ChnId = 0;
    ret = RK_MPI_SYS_Bind(&vi_channel, &rga_channel);
    if (ret != 0)
    {
        printf("vi bind rga error\n");
        return -1;
    }
    else
    {
        printf("vi bind rga success\n");
    }

    pthread_t pid;
    //VENC线程的参数
    VENC_PROC_PARAM *venc_arg_params = (VENC_PROC_PARAM *)malloc(sizeof(VENC_PROC_PARAM));
    if (venc_arg_params == NULL)
    {
        printf("malloc venc arg error\n");
        free(venc_arg_params);
    }

    venc_arg_params->vencId = venc_channel.s32ChnId;
    //创建VENC线程，获取摄像头编码数据
    ret = pthread_create(&pid, NULL, camera_venc_thread, (void *)venc_arg_params);
    if (ret != 0)
    {
        printf("create camera_venc_thread failed\n");
    }

    ret = pthread_create(&pid, NULL, get_rga_thread, NULL);
    if(ret != 0)
    {
        printf("create get_rga_thread failed\n");
    }

    //VENC线程的参数
    VENC_PROC_PARAM *low_venc_arg_params = (VENC_PROC_PARAM *)malloc(sizeof(VENC_PROC_PARAM));
    if (venc_arg_params == NULL)
    {
        printf("malloc venc arg error\n");
        free(venc_arg_params);
    }

    low_venc_arg_params->vencId = low_venc_channel.s32ChnId;
    //创建VENC线程，获取摄像头编码数据
    ret = pthread_create(&pid, NULL, low_camera_venc_thread, (void *)low_venc_arg_params);
    if (ret != 0)
    {
        printf("create camera_venc_thread failed\n");
    }

    //创建HIGH_PUSH线程
    ret = pthread_create(&pid, NULL, high_video_push_thread, (void *)ffmpeg_config);
    if (ret != 0)
    {
        printf("push_server_thread error\n");
    }

    //创建LOW_PUSH线程
    ret = pthread_create(&pid, NULL, low_video_push_thread, (void *)low_ffmpeg_config);
    if (ret != 0)
    {
        printf("push_server_thread error\n");
    }

    return 0;
}
