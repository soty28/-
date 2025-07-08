#include <rtthread.h>
#include <dfs_posix.h>
#include "ecat_slave.h"

#define MAX_TRAJECTORY_POINTS 1800 // 5分钟@6Hz
#define TRAJ_FILE_PREFIX "/flash/traj_"

// 轨迹点结构
typedef struct {
    float joint_angles[6];  // 6个关节角度
    float gripper_state;    // 夹爪状态
    uint32_t timestamp;     // 时间戳
} TrajectoryPoint;

static TrajectoryPoint trajectory[MAX_TRAJECTORY_POINTS];
static int traj_index = 0;
static int replay_index = 0;
static int current_episode_id = -1;

// 初始化轨迹系统
void trajectory_init(int mode)
{
    traj_index = 0;
    replay_index = 0;
    
    if (mode == RECORD_MODE) {
        // 为新录制分配 episode ID
        current_episode_id = find_next_episode_id();
        rt_kprintf("Recording to episode %d\n", current_episode_id);
    }
}

// 录制轨迹点
void record_trajectory_point(void)
{
    if (traj_index >= MAX_TRAJECTORY_POINTS) {
        rt_kprintf("Trajectory buffer full!\n");
        return;
    }
    
    // 获取当前关节角度
    get_joint_positions(trajectory[traj_index].joint_angles);
    
    // 获取夹爪状态
    trajectory[traj_index].gripper_state = get_gripper_state();
    
    // 时间戳
    trajectory[traj_index].timestamp = rt_tick_get();
    
    traj_index++;
}

// 录制步骤
void record_trajectory_step(void)
{
    record_trajectory_point();
    
    // 控制录制频率 6Hz
    rt_thread_mdelay(166);
    
    // 检查录制结束条件
    if (is_object_grasped() || traj_index >= MAX_TRAJECTORY_POINTS) {
        save_trajectory();
        current_state = STATE_IDLE;
        rt_kprintf("Trajectory recorded: %d points\n", traj_index);
    }
}

// 保存轨迹到Flash
void save_trajectory(void)
{
    char filename[32];
    rt_snprintf(filename, sizeof(filename), "%s%d.bin", TRAJ_FILE_PREFIX, current_episode_id);
    
    int fd = open(filename, O_WRONLY | O_CREAT);
    if (fd < 0) {
        rt_kprintf("Failed to open file %s\n", filename);
        return;
    }
    
    // 写入轨迹数据
    write(fd, &traj_index, sizeof(int));
    write(fd, trajectory, traj_index * sizeof(TrajectoryPoint));
    
    close(fd);
    rt_kprintf("Trajectory saved to %s\n", filename);
}

// 加载轨迹
int trajectory_load(int episode_id)
{
    char filename[32];
    rt_snprintf(filename, sizeof(filename), "%s%d.bin", TRAJ_FILE_PREFIX, episode_id);
    
    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        rt_kprintf("File not found: %s\n", filename);
        return -1;
    }
    
    // 读取轨迹点数
    read(fd, &traj_index, sizeof(int));
    
    // 读取轨迹数据
    read(fd, trajectory, traj_index * sizeof(TrajectoryPoint));
    
    close(fd);
    
    current_episode_id = episode_id;
    replay_index = 0;
    rt_kprintf("Loaded trajectory %d with %d points\n", episode_id, traj_index);
    return 0;
}

// 回放步骤
void replay_trajectory_step(void)
{
    if (replay_index >= traj_index) {
        // 回放完成
        current_state = STATE_IDLE;
        rt_kprintf("Trajectory replay completed\n");
        return;
    }
    
    // 设置关节角度
    set_joint_positions(trajectory[replay_index].joint_angles);
    
    // 设置夹爪状态
    set_gripper_state(trajectory[replay_index].gripper_state);
    
    // 控制回放速度 (基于时间戳)
    if (replay_index > 0) {
        uint32_t delta_t = trajectory[replay_index].timestamp - 
                          trajectory[replay_index-1].timestamp;
        rt_thread_mdelay(delta_t);
    } else {
        rt_thread_mdelay(10);
    }
    
    replay_index++;
}