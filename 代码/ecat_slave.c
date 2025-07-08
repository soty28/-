#include <rtthread.h>
#include "ecat_slave.h"

// EtherCAT PDO 映射
typedef struct {
    // 输入PDO (从主站接收)
    struct {
        int32_t target_pos[6];    // 6关节目标位置
        uint8_t gripper_cmd;      // 夹爪控制命令
    } rx_pdo;
    
    // 输出PDO (发送到主站)
    struct {
        int32_t actual_pos[6];    // 6关节实际位置
        uint8_t gripper_state;    // 夹爪状态
        uint8_t status;           // 设备状态
    } tx_pdo;
} EcataPdoMapping;

static EcataPdoMapping ecat_pdo;

// 初始化 EtherCAT 从站
void ecat_slave_init(void)
{
    // 初始化 EtherCAT 协议栈
    ecat_init();
    
    // 配置 PDO 映射
    ecat_configure_pdo_mapping(sizeof(ecat_pdo));
    
    rt_kprintf("EtherCAT slave initialized\n");
}

// 处理 EtherCAT 通信
void ecat_process(void)
{
    // 1. 接收主站数据
    ecat_receive();
    
    // 2. 更新控制输出
    for (int i = 0; i < 6; i++) {
        set_joint_target(i, ecat_pdo.rx_pdo.target_pos[i]);
    }
    set_gripper_command(ecat_pdo.rx_pdo.gripper_cmd);
    
    // 3. 更新反馈数据
    for (int i = 0; i < 6; i++) {
        ecat_pdo.tx_pdo.actual_pos[i] = get_joint_position(i);
    }
    ecat_pdo.tx_pdo.gripper_state = get_gripper_state();
    ecat_pdo.tx_pdo.status = get_device_status();
    
    // 4. 发送数据到主站
    ecat_send();
    
    // 控制频率 1kHz
    rt_thread_mdelay(1);
}