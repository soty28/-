#include <rtthread.h>
#include <rtdevice.h>
#include "ecat_slave.h"

#define SERIAL_DEVICE_NAME "uart2"  // 连接GD32机械臂的串口

static rt_device_t serial_dev;
static struct rt_semaphore rx_sem;

// 串口接收回调
static rt_err_t uart_rx_callback(rt_device_t dev, rt_size_t size) {
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

// 初始化串口
int usb_serial_init(void) {
    serial_dev = rt_device_find(SERIAL_DEVICE_NAME);
    if (!serial_dev) {
        rt_kprintf("Serial device %s not found!\n", SERIAL_DEVICE_NAME);
        return -RT_ERROR;
    }
    
    // 打开串口设备
    if (rt_device_open(serial_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        rt_kprintf("Failed to open serial device\n");
        return -RT_ERROR;
    }
    
    // 创建信号量
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    
    // 设置接收回调
    rt_device_set_rx_indicate(serial_dev, uart_rx_callback);
    
    // 配置串口参数
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(serial_dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_kprintf("USB-Serial initialized\n");
    return RT_EOK;
}

// 串口数据处理线程
static void serial_thread_entry(void *parameter) {
    uint8_t rx_buffer[64];
    ArmCommand arm_cmd;
    ArmStatus arm_status;
    
    while (1) {
        // 等待串口数据
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
        // 读取数据
        rt_size_t size = rt_device_read(serial_dev, 0, rx_buffer, sizeof(rx_buffer));
        
        // 处理接收到的数据
        for (int i = 0; i < size; i++) {
            static int state = 0;
            static int index = 0;
            static uint8_t buffer[sizeof(ArmStatus)];
            
            switch (state) {
                case 0: // 等待帧头
                    if (rx_buffer[i] == 0xCC) {
                        state = 1;
                        buffer[0] = 0xCC;
                        index = 1;
                    }
                    break;
                    
                case 1: // 检查第二个字节
                    if (rx_buffer[i] == 0xDD) {
                        buffer[1] = 0xDD;
                        state = 2;
                        index = 2;
                    } else {
                        state = 0;
                    }
                    break;
                    
                case 2: // 接收完整帧
                    buffer[index++] = rx_buffer[i];
                    if (index >= sizeof(ArmStatus)) {
                        // 复制到结构体
                        rt_memcpy(&arm_status, buffer, sizeof(ArmStatus));
                        
                        // 验证校验和
                        uint8_t checksum = 0;
                        for (int j = 0; j < sizeof(ArmStatus) - 1; j++) {
                            checksum += buffer[j];
                        }
                        
                        if (checksum == arm_status.checksum) {
                            // 更新EtherCAT输入PDO
                            for (int j = 0; j < 6; j++) {
                                ecat_pdo.tx_pdo.actual_pos[j] = 
                                    (int32_t)(arm_status.joint_angles[j] * 1000000); // 转换为微弧度
                            }
                            ecat_pdo.tx_pdo.gripper_state = arm_status.gripper_state;
                        }
                        
                        state = 0;
                    }
                    break;
            }
        }
    }
}

// 发送控制命令到机械臂
void send_arm_command(const float *joint_angles, uint8_t gripper) {
    ArmCommand cmd = {
        .header = {0xAA, 0xBB},
        .gripper = gripper
    };
    
    // 设置关节角度
    for (int i = 0; i < 6; i++) {
        cmd.joint_angles[i] = joint_angles[i];
    }
    
    // 计算校验和
    cmd.checksum = 0;
    uint8_t *data = (uint8_t*)&cmd;
    for (int i = 0; i < sizeof(cmd) - 1; i++) {
        cmd.checksum += data[i];
    }
    
    // 发送命令
    rt_device_write(serial_dev, 0, data, sizeof(cmd));
}

// 初始化串口通信
int arm_communication_init(void) {
    if (usb_serial_init() != RT_EOK) {
        return -1;
    }
    
    // 创建串口处理线程
    rt_thread_t thread = rt_thread_create("serial",
                                         serial_thread_entry,
                                         RT_NULL,
                                         2048,
                                         20,
                                         10);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    }
    
    return 0;
}