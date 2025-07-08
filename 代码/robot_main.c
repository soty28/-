#include <rtthread.h>
#include "robot_control.h"
#include "ecat_slave.h"
#include "camera.h"
#include "rl_train.h"
#include "trajectory.h"

// 全局状态机
enum SystemState {
    STATE_IDLE,
    STATE_TRAINING,
    STATE_RECORDING,
    STATE_REPLAY
};

static enum SystemState current_state = STATE_IDLE;
static int training_episodes = 0;
static int current_episode = 0;

// 机械臂控制线程
static void robot_control_thread(void *param)
{
    while (1) {
        // 处理 EtherCAT 通信
        ecat_process();
        
        switch (current_state) {
            case STATE_TRAINING:
                rl_train_step();
                break;
                
            case STATE_RECORDING:
                record_trajectory_step();
                break;
                
            case STATE_REPLAY:
                replay_trajectory_step();
                break;
                
            default:
                // IDLE 状态不执行操作
                rt_thread_mdelay(10);
                break;
        }
    }
}

// 启动训练任务
int start_training(int episodes)
{
    if (current_state != STATE_IDLE) {
        rt_kprintf("System busy, cannot start training\n");
        return -1;
    }
    
    rl_init(); // 初始化强化学习模型
    training_episodes = episodes;
    current_episode = 0;
    current_state = STATE_TRAINING;
    
    rt_kprintf("Training started for %d episodes\n", episodes);
    return 0;
}

// 录制轨迹
int start_recording(void)
{
    if (current_state != STATE_IDLE) {
        rt_kprintf("System busy, cannot start recording\n");
        return -1;
    }
    
    trajectory_init(RECORD_MODE);
    current_state = STATE_RECORDING;
    rt_kprintf("Trajectory recording started\n");
    return 0;
}

// 回放轨迹
int start_replay(int episode_id)
{
    if (current_state != STATE_IDLE) {
        rt_kprintf("System busy, cannot start replay\n");
        return -1;
    }
    
    if (trajectory_load(episode_id) != 0) {
        rt_kprintf("Failed to load episode %d\n", episode_id);
        return -1;
    }
    
    current_state = STATE_REPLAY;
    rt_kprintf("Replaying episode %d\n", episode_id);
    return 0;
}

// 停止当前任务
void stop_current_task(void)
{
    current_state = STATE_IDLE;
    rt_kprintf("Task stopped\n");
}

// 命令行接口
static void robot_cmd(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage:\n");
        rt_kprintf("robot train <episodes> - Start training\n");
        rt_kprintf("robot record          - Record trajectory\n");
        rt_kprintf("robot replay <id>     - Replay trajectory\n");
        rt_kprintf("robot stop            - Stop current task\n");
        return;
    }
    
    if (rt_strcmp(argv[1], "train") == 0 && argc >= 3) {
        start_training(atoi(argv[2]));
    } 
    else if (rt_strcmp(argv[1], "record") == 0) {
        start_recording();
    }
    else if (rt_strcmp(argv[1], "replay") == 0 && argc >= 3) {
        start_replay(atoi(argv[2]));
    }
    else if (rt_strcmp(argv[1], "stop") == 0) {
        stop_current_task();
    }
    else {
        rt_kprintf("Unknown command\n");
    }
}
MSH_CMD_EXPORT(robot_cmd, Robot control commands);

// 初始化函数
int robot_main_init(void)
{
    // 初始化硬件
    camera_init();
    ecat_slave_init();
    servo_init();
    
    // 创建控制线程
    rt_thread_t tid = rt_thread_create("robot_ctrl", 
                                      robot_control_thread, 
                                      RT_NULL, 
                                      2048, 
                                      10, 
                                      10);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
    }
    
    rt_kprintf("Robot control system initialized\n");
    return 0;
}
INIT_APP_EXPORT(robot_main_init);