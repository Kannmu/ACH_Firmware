#pragma once
#define _USE_MATH_DEFINES

#include "main.h"
#include "transducer.h"
#include "usbd_cdc_if.h"

// 协议定义
#define FRAME_HEADER_1          0xAA
#define FRAME_HEADER_2          0x55
#define FRAME_TAIL_1            0x0D
#define FRAME_TAIL_2            0x0A

// 命令类型 (PC -> UMH)
#define CMD_SET_POINT           0x01
#define CMD_ENABLE_DISABLE      0x02
#define CMD_GET_STATUS          0x03
#define CMD_PING                0x04

// 响应类型 (UMH -> PC)
#define RSP_ACK                 0x80
#define RSP_NACK                0x81
#define RSP_RETURN_STATUS       0x82
#define RSP_PING_ACK            0x83
#define RSP_ERROR_CODE          0xFF

// 协议帧结构
typedef struct {
    uint8_t header[2];      // 帧头 0xAA 0x55
    uint8_t cmd_type;       // 命令/消息类型
    uint8_t data_length;    // 数据长度
    uint8_t data[255];      // 数据载荷
    uint8_t checksum;       // 校验和
    uint8_t tail[2];        // 帧尾 0x0D 0x0A
} comm_frame_t;

// 接收状态枚举
typedef enum {
    RX_STATE_WAIT_HEADER1,
    RX_STATE_WAIT_HEADER2,
    RX_STATE_WAIT_CMD_TYPE,
    RX_STATE_WAIT_DATA_LENGTH,
    RX_STATE_WAIT_DATA,
    RX_STATE_WAIT_CHECKSUM,
    RX_STATE_WAIT_TAIL1,
    RX_STATE_WAIT_TAIL2,
    RX_STATE_FRAME_COMPLETE
} rx_state_t;

// 接收缓冲区结构
typedef struct {
    rx_state_t state;
    comm_frame_t frame;
    uint8_t data_index;
    uint8_t calculated_checksum;
} rx_buffer_t;

// 函数声明
void Comm_Init(void);
void Comm_Process_Received_Data(uint8_t* data, uint32_t length);
void Comm_Send_Response(uint8_t cmd_type, uint8_t* data, uint8_t data_length);
uint8_t Comm_Calculate_Checksum(uint8_t cmd_type, uint8_t data_length, uint8_t* data);
void Comm_Handle_Ping_Command(uint8_t* data, uint8_t data_length);
void Comm_Reset_Rx_State(void);