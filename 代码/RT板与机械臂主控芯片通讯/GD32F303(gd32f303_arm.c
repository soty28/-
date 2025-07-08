#include "gd32f30x.h"
#include <stdio.h>

// 机械臂控制指令结构体
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // 帧头 0xAA, 0xBB
    float joint_angles[6];  // 6个关节角度 (弧度)
    uint8_t gripper;        // 夹爪状态 (0-255)
    uint8_t checksum;       // 校验和
} ArmCommand;
#pragma pack(pop)

// 机械臂状态反馈结构体
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // 帧头 0xCC, 0xDD
    float joint_angles[6];  // 实际关节角度
    uint8_t gripper_state;  // 夹爪实际状态
    uint8_t status;         // 状态字
    uint8_t checksum;       // 校验和
} ArmStatus;
#pragma pack(pop)

void uart_init(uint32_t baudrate) {
    // 初始化UART1 (连接CH340T)
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_GPIOA);
    
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9); // TX
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10); // RX
    
    usart_deinit(USART1);
    usart_baudrate_set(USART1, baudrate);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
}

void send_arm_status(void) {
    ArmStatus status = {
        .header = {0xCC, 0xDD},
        .gripper_state = read_gripper_position(),
        .status = 0x01  // 正常运行状态
    };
    
    // 读取关节实际角度
    for (int i = 0; i < 6; i++) {
        status.joint_angles[i] = read_joint_angle(i);
    }
    
    // 计算校验和
    status.checksum = 0;
    uint8_t *data = (uint8_t*)&status;
    for (int i = 0; i < sizeof(status) - 1; i++) {
        status.checksum += data[i];
    }
    
    // 发送状态数据
    for (int i = 0; i < sizeof(status); i++) {
        usart_data_transmit(USART1, data[i]);
        while (usart_flag_get(USART1, USART_FLAG_TBE) == RESET);
    }
}

void process_uart_command(void) {
    static uint8_t rx_buf[64];
    static int index = 0;
    
    if (usart_flag_get(USART1, USART_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART1);
        rx_buf[index++] = data;
        
        // 检查帧头
        if (index >= 2) {
            if (rx_buf[0] == 0xAA && rx_buf[1] == 0xBB) {
                // 检查完整帧
                if (index == sizeof(ArmCommand)) {
                    ArmCommand *cmd = (ArmCommand*)rx_buf;
                    
                    // 验证校验和
                    uint8_t checksum = 0;
                    for (int i = 0; i < sizeof(ArmCommand) - 1; i++) {
                        checksum += rx_buf[i];
                    }
                    
                    if (checksum == cmd->checksum) {
                        // 执行控制指令
                        for (int i = 0; i < 6; i++) {
                            set_joint_angle(i, cmd->joint_angles[i]);
                        }
                        set_gripper_position(cmd->gripper);
                        
                        // 发送状态反馈
                        send_arm_status();
                    }
                    
                    index = 0;
                }
            } else {
                index = 0; // 无效帧头，重置
            }
        }
        
        // 防止缓冲区溢出
        if (index >= sizeof(rx_buf)) {
            index = 0;
        }
    }
}

int main(void) {
    // 系统时钟初始化
    rcu_clock_freq_set(RCU_CKSYSSRC_PLL, RCU_PLL_MUL12);
    SystemCoreClockUpdate();
    
    // 初始化机械臂硬件
    arm_hardware_init();
    uart_init(115200); // 设置串口波特率
    
    while (1) {
        process_uart_command();
        // 其他控制逻辑...
    }
}