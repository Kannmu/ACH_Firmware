#define _USE_MATH_DEFINES
#include "transducer.h"
#include "dma_manager.h"
#include "communication.h"
#include "utiles.h"

// 全局变量
static rx_buffer_t rx_buffer;

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void)
{
    Comm_Reset_Rx_State();
}

/**
 * @brief 重置接收状态
 */
void Comm_Reset_Rx_State(void)
{
    rx_buffer.state = RX_STATE_WAIT_HEADER1;
    rx_buffer.data_index = 0;
    rx_buffer.calculated_checksum = 0;
    memset(&rx_buffer.frame, 0, sizeof(comm_frame_t));
}

/**
 * @brief 计算校验和
 * @param cmd_type 命令类型
 * @param data_length 数据长度
 * @param data 数据指针
 * @return 校验和
 */
uint8_t Comm_Calculate_Checksum(uint8_t cmd_type, uint8_t data_length, uint8_t* data)
{
    uint16_t sum = cmd_type + data_length;
    
    for (uint8_t i = 0; i < data_length; i++) {
        sum += data[i];
    }
    
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief 发送响应帧
 * @param cmd_type 响应类型
 * @param data 数据指针
 * @param data_length 数据长度
 */
void Comm_Send_Response(uint8_t cmd_type, uint8_t* data, uint8_t data_length)
{
    uint8_t tx_buffer[260]; // 最大帧长度
    uint8_t index = 0;
    
    // 帧头
    tx_buffer[index++] = FRAME_HEADER_1;
    tx_buffer[index++] = FRAME_HEADER_2;
    
    // 命令类型
    tx_buffer[index++] = cmd_type;
    
    // 数据长度
    tx_buffer[index++] = data_length;
    
    // 数据载荷
    if (data_length > 0 && data != NULL) {
        memcpy(&tx_buffer[index], data, data_length);
        index += data_length;
    }
    
    // 校验和
    tx_buffer[index++] = Comm_Calculate_Checksum(cmd_type, data_length, data);
    
    // 帧尾
    tx_buffer[index++] = FRAME_TAIL_1;
    tx_buffer[index++] = FRAME_TAIL_2;
    
    // 通过USB CDC发送
    CDC_Transmit_FS(tx_buffer, index);
}

/**
 * @brief 处理Ping命令
 * @param data 接收到的数据
 * @param data_length 数据长度
 */
void Comm_Handle_Ping_Command(uint8_t* data, uint8_t data_length)
{
    // 将接收到的随机数原样返回
    Comm_Send_Response(RSP_PING_ACK, data, data_length);
}

/**
 * @brief 处理接收到的数据
 * @param data 接收到的数据缓冲区
 * @param length 数据长度
 */
void Comm_Process_Received_Data(uint8_t* data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        
        switch (rx_buffer.state) {
            case RX_STATE_WAIT_HEADER1:
                if (byte == FRAME_HEADER_1) {
                    rx_buffer.frame.header[0] = byte;
                    rx_buffer.state = RX_STATE_WAIT_HEADER2;
                }
                break;
                
            case RX_STATE_WAIT_HEADER2:
                if (byte == FRAME_HEADER_2) {
                    rx_buffer.frame.header[1] = byte;
                    rx_buffer.state = RX_STATE_WAIT_CMD_TYPE;
                } else {
                    Comm_Reset_Rx_State();
                }
                break;
                
            case RX_STATE_WAIT_CMD_TYPE:
                rx_buffer.frame.cmd_type = byte;
                rx_buffer.calculated_checksum = byte;
                rx_buffer.state = RX_STATE_WAIT_DATA_LENGTH;
                break;
                
            case RX_STATE_WAIT_DATA_LENGTH:
                rx_buffer.frame.data_length = byte;
                rx_buffer.calculated_checksum += byte;
                rx_buffer.data_index = 0;
                
                if (rx_buffer.frame.data_length == 0) {
                    rx_buffer.state = RX_STATE_WAIT_CHECKSUM;
                } else {
                    rx_buffer.state = RX_STATE_WAIT_DATA;
                }
                break;
                
            case RX_STATE_WAIT_DATA:
                rx_buffer.frame.data[rx_buffer.data_index] = byte;
                rx_buffer.calculated_checksum += byte;
                rx_buffer.data_index++;
                
                if (rx_buffer.data_index >= rx_buffer.frame.data_length) {
                    rx_buffer.state = RX_STATE_WAIT_CHECKSUM;
                }
                break;
                
            case RX_STATE_WAIT_CHECKSUM:
                rx_buffer.frame.checksum = byte;
                
                // 验证校验和
                if ((rx_buffer.calculated_checksum & 0xFF) == byte) {
                    rx_buffer.state = RX_STATE_WAIT_TAIL1;
                } else {
                    // 校验和错误，重置状态
                    Comm_Reset_Rx_State();
                }
                break;
                
            case RX_STATE_WAIT_TAIL1:
                if (byte == FRAME_TAIL_1) {
                    rx_buffer.frame.tail[0] = byte;
                    rx_buffer.state = RX_STATE_WAIT_TAIL2;
                } else {
                    Comm_Reset_Rx_State();
                }
                break;
                
            case RX_STATE_WAIT_TAIL2:
                if (byte == FRAME_TAIL_2) {
                    rx_buffer.frame.tail[1] = byte;
                    rx_buffer.state = RX_STATE_FRAME_COMPLETE;
                    
                    // 处理完整的帧
                    switch (rx_buffer.frame.cmd_type) {
                        case CMD_PING:
                            Comm_Handle_Ping_Command(rx_buffer.frame.data, rx_buffer.frame.data_length);
                            break;
                        case CMD_SET_POINT:
                        case CMD_ENABLE_DISABLE:
                        case CMD_GET_STATUS:
                        {
                            // Data: Voltage, Temperature
                            // Byte Structure: 1 bit: IsEnabled(1 byte), float: Voltage(4 bytes), float: Temperature(4 bytes)
                            float voltage, temperature;
                            voltage = Get_Voltage();
                            temperature = Get_Temperature();

                            // 打包数据
                            uint8_t response_data[8];
                            memcpy(response_data, &voltage, sizeof(voltage));
                            memcpy(response_data + 4, &temperature, sizeof(temperature));

                            Comm_Send_Response(RSP_RETURN_STATUS, response_data, 8);
                            break;
                        }
                        default:
                            // 未知命令，返回NACK
                            Comm_Send_Response(RSP_NACK, NULL, 0);
                            break;
                    }
                }
                
                // 处理完成后重置状态
                Comm_Reset_Rx_State();
                break;
                
            case RX_STATE_FRAME_COMPLETE:
                // 这个状态不应该到达，重置状态
                Comm_Reset_Rx_State();
                break;
        }
    }
}