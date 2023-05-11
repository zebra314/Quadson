#ifndef INC_CAN_INTERFACE_H
#define INC_CAN_INTERFACE_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <list>

/* prama define discrible -----------------------------------------------------*/
#define CAN_RX_MSG_BUF ((int)128)
#define CAN_TX_MSG_BUF ((int)128)
#define CAN_MSG_LEN ((int)8)

/* strcuct  discrible --------------------------------------------------------*/
typedef void (*callback_fn)(int, uint8_t *, int);

/* can 消息结构体*/
typedef struct
{
    uint16_t id;
    uint16_t time_stamp;
    uint8_t time_flag;
    uint8_t send_type;
    uint8_t remote_type;
    uint8_t extern_flag;
    uint8_t data_len;
    uint8_t data_buf[CAN_MSG_LEN];
    uint8_t reserved[3];
} can_obj_t;

/* 设备结构体*/
typedef struct
{
    int sock_fd;
    int epoll_fd;
    callback_fn recive_callback;
} can_device_t;

typedef struct
{
    uint8_t id;
    uint8_t cmd;
    int16_t value;
} can_cmd_t;

extern std::list<can_cmd_t> can_cmd_buffer;

class Can_interface
{
private:
    int mSocket;
    pthread_t mCan_thread = -1;
    can_device_t mVcuCan;
    std::string mCanName = "can0";

public:
    int can_init();
    void can_send_cmd(int motor_index, int cmd, int value);
    void can_open();
    void can_close();

    Can_interface();
    ~Can_interface();
};

#endif
