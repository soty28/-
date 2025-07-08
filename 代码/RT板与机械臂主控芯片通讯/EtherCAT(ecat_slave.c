#include <rtthread.h>
#include "ecat_slave.h"
#include "arm_communication.h"

// EtherCAT PDO 映射
typedef struct {
    // 输入PDO (从主站接收)
    struct {
        int32_t target_pos[6];    // 6关节目标位置 (微弧度)
        uint8_t gripper_cmd;      // 夹爪控制命令
    } rx_pdo;
    
    // 输出PDO (发送到主站)
    struct {
        int32_t actual_pos[6];    // 6关节实际位置 (微弧度)
        uint8_t gripper_state;    // 夹爪状态
        uint8_t status;           // 设备状态
    } tx_pdo;
} EcataPdoMapping;

static EcataPdoMapping ecat_pdo;

// 初始化 EtherCAT 从站
void ecat_slave_init(void) {
    // 初始化 EtherCAT 协议栈
    ecat_init();
    
    // 配置 PDO 映射
    ecat_configure_pdo_mapping(sizeof(ecat_pdo));
    
    // 初始化机械臂通信
    arm_communication_init();
    
    rt_kprintf("EtherCAT slave initialized\n");
}

// 处理 EtherCAT 通信
void ecat_process(void) {
    // 1. 接收主站数据
    ecat_receive();
    
    // 2. 将目标位置发送到教师机械臂
    float joint_angles[6];
    for (int i = 0; i < 6; i++) {
        joint_angles[i] = ecat_pdo.rx_pdo.target_pos[i] / 1000000.0f; // 转换为弧度
    }
    send_arm_command(joint_angles, ecat_pdo.rx_pdo.gripper_cmd);
    
    // 3. 更新反馈数据 (由串口线程异步更新)
    // tx_pdo 已在串口线程中更新
    
    // 4. 发送数据到主站
    ecat_send();
    
    // 控制频率 100Hz
    rt_thread_mdelay(10);
}