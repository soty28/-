#include <rtthread.h>
#include <math.h>
#include "neural_net.h"
#include "camera.h"
#include "robot_kinematics.h"

#define STATE_SIZE  (64*64 + 6)  // 图像特征 + 关节角度
#define ACTION_SIZE 6            // 6个关节的动作
#define MAX_EPISODE_STEPS 300    // 5分钟@10Hz

// 轻量级神经网络模型
static NeuralNet policy_net;

// 训练状态
static float current_state[STATE_SIZE];
static int episode_step = 0;
static float total_reward = 0.0f;

// 初始化强化学习
void rl_init(void)
{
    // 创建神经网络: 输入层->隐藏层->输出层
    nn_init(&policy_net, STATE_SIZE, 32, ACTION_SIZE);
    
    // 加载预训练权重 (如果存在)
    if (nn_load_weights(&policy_net, "/flash/rl_model.bin") != 0) {
        rt_kprintf("Initializing new RL model\n");
        nn_random_init(&policy_net);
    }
}

// 获取当前状态
void get_current_state(float *state)
{
    // 获取图像特征 (简化处理)
    float img_features[64*64];
    camera_capture_simplified(img_features);
    
    // 获取关节角度
    float joint_angles[6];
    get_joint_positions(joint_angles);
    
    // 合并状态向量
    rt_memcpy(state, img_features, sizeof(float)*64*64);
    rt_memcpy(state + 64*64, joint_angles, sizeof(float)*6);
}

// 计算奖励函数
float calculate_reward(void)
{
    // 简化的奖励函数:
    // 1. 夹爪接近目标物体
    // 2. 成功抓取物体
    // 3. 节能奖励
    
    float reward = 0.0f;
    float gripper_pos[3];
    get_gripper_position(gripper_pos);
    
    // 目标物体位置 (简化处理)
    float target_pos[3] = {0.3, 0.2, 0.1};
    
    // 距离奖励
    float dist = sqrtf(powf(gripper_pos[0]-target_pos[0], 2) +
                 powf(gripper_pos[1]-target_pos[1], 2) +
                 powf(gripper_pos[2]-target_pos[2], 2);
    
    reward += 0.1f * (1.0f - dist);
    
    // 抓取成功奖励 (简化)
    if (is_object_grasped()) {
        reward += 10.0f;
    }
    
    return reward;
}

// 执行训练步骤
void rl_train_step(void)
{
    if (episode_step == 0) {
        // 新回合开始
        get_current_state(current_state);
        total_reward = 0.0f;
        rt_kprintf("Episode %d started\n", current_episode);
    }
    
    // 1. 通过策略网络选择动作
    float action[ACTION_SIZE];
    nn_forward(&policy_net, current_state, action);
    
    // 2. 执行动作 (转换为关节角度变化)
    float joint_deltas[6];
    for (int i = 0; i < 6; i++) {
        // 动作值映射到 [-5°, +5°]
        joint_deltas[i] = (action[i] - 0.5f) * 10.0f;
    }
    move_joints_delta(joint_deltas);
    
    // 3. 获取新状态和奖励
    float new_state[STATE_SIZE];
    get_current_state(new_state);
    float reward = calculate_reward();
    total_reward += reward;
    
    // 4. 更新神经网络 (简化版DQN)
    // TODO: 实现经验回放和目标网络
    
    // 5. 检查回合结束条件
    episode_step++;
    if (episode_step >= MAX_EPISODE_STEPS || is_object_grasped()) {
        rt_kprintf("Episode %d finished. Total reward: %.2f\n", 
                  current_episode, total_reward);
        
        // 保存模型每10轮
        if (current_episode % 10 == 0) {
            nn_save_weights(&policy_net, "/flash/rl_model.bin");
        }
        
        // 准备下一回合
        reset_environment();
        current_episode++;
        episode_step = 0;
        
        // 检查训练完成
        if (current_episode >= training_episodes) {
            current_state = STATE_IDLE;
            rt_kprintf("Training completed!\n");
            
            // 保存最终模型
            nn_save_weights(&policy_net, "/flash/rl_final_model.bin");
        }
    } else {
        // 更新状态
        rt_memcpy(current_state, new_state, sizeof(current_state));
    }
    
    // 控制频率 10Hz
    rt_thread_mdelay(100);
}