#include "can_interface.h"
#include "utility.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define ID_STD_OFFSET 6
#define ID_EXT_OFFSET 24
#define DEBUG
#define EPOLL_SIZE 1024

static pthread_mutex_t recive_lock;
std::list<can_cmd_t> can_cmd_buffer;

int add_epoll_fd(int sock_fd) {
  int epfd;
  struct epoll_event event;

  /*创建epoll模型*/
  epfd = epoll_create(EPOLL_SIZE);
  if (epfd < 0) {
    return -1;
  }

  /*监听sock_fd 的可读事件*/
  event.data.fd = sock_fd;
  event.events = EPOLLIN;
  if (epoll_ctl(epfd, EPOLL_CTL_ADD, sock_fd, &event) < 0) {
    return -1;
  }

  return epfd;
}

void can_recive(int can_id, uint8_t *buf, int buf_len) {

  int isGroup = can_id >> 10;
  uint8_t motor_id = can_id >> 6 & 0x0F;
  uint8_t cmd_type = can_id & 0x3F;
  int16_t value = MAKE_SHORT(buf[0], buf[1]);

  can_cmd_t cmd = {
      .id = motor_id,
      .cmd = cmd_type,
      .value = value,
  };
  can_cmd_buffer.push_back(cmd);

  printf("Motor: %d, Cmd: 0x%02X, LEN: %02d\t", motor_id, cmd_type,
                        buf_len);
  for (int i = 0; i < buf_len; i++) {
    printf("%02X  ", *(buf + i));
  }
  printf("\r\n");
}

void *can_recive_thread(void *param) {
  int i, nfds;
  int timeout = 2;
  uint64_t nbytes;
  struct can_frame rx_frame;
  struct epoll_event events[EPOLL_SIZE];
  can_device_t *can_device_temp = (can_device_t *)param;

  while (1) {
    nfds = epoll_wait(can_device_temp->epoll_fd, events, EPOLL_SIZE, timeout);
    if (nfds < 0) {
      //   printf(">>:  epoll wait error!\r\n");
    }
    for (int i = 0; i < nfds; i++) {
      if (events[i].events & EPOLLIN) {
        nbytes = read(events[i].data.fd, &rx_frame, sizeof(rx_frame));
        if (nbytes > 0) {
          if (can_device_temp->recive_callback) {
            can_device_temp->recive_callback(rx_frame.can_id, rx_frame.data,
                                             rx_frame.can_dlc);
          }
        }
      }
    }
  }
}
int Can_interface::can_init() {
  int can_fd;
  struct ifreq ifr;
  struct sockaddr_can addr;
  /*创建套接字*/
  can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd < 0) {
    return -1;
  }
  /*指定 can 设备*/
  strcpy(ifr.ifr_name, mCanName.c_str());
  ioctl(can_fd, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /*关闭回环模式*/
  int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
  setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback,
             sizeof(loopback));

  /*关闭自收自发*/
  int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
  setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs,
             sizeof(recv_own_msgs));

  /*将套接字与 can0 绑定*/
  bind(can_fd, (struct sockaddr *)&addr, sizeof(addr));

  /*bind can 端口*/
  mVcuCan.sock_fd = can_fd;
  if (mVcuCan.sock_fd < 0) {
    printf(">>: can socket create failed!\r\n");
    return -1;
  }

  /*开启epoll事件监听*/
  mVcuCan.epoll_fd = add_epoll_fd(mVcuCan.sock_fd);
  if (mVcuCan.epoll_fd < 0) {
    close(mVcuCan.sock_fd);
    printf(">>: can socket epoll create failed!\r\n");
    return -1;
  }
  /**注册接收回调函数*/
  if (can_recive) {
    mVcuCan.recive_callback = can_recive;
  }

  printf(">>: can init success, sock_fd:  %d, epoll_fd: %d !\r\n",
         mVcuCan.sock_fd, mVcuCan.epoll_fd);
  return 1;
}

void Can_interface::can_send_cmd(int motor_index, int cmd, int value) {
  struct can_frame frame;
  int nbytes;

  bzero(&frame, sizeof(frame));

  frame.can_id = motor_index << ID_STD_OFFSET | cmd;
  frame.can_dlc = 2;
  frame.data[0] = HIGH_BYTE(value);
  frame.data[1] = LOW_BYTE(value);
  nbytes = write(mVcuCan.sock_fd, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {
#ifdef DEBUG
    printf("Wrote %d bytes\n", nbytes);
#endif
  }
  // return tran_count_ret;
}

void Can_interface::can_open() {
  /*创建接收进程*/
  pthread_mutex_init(&recive_lock, NULL);
  pthread_create(&mCan_thread, NULL, can_recive_thread, &mVcuCan);
  // printf(">>: epoll thread fd: %d\r\n", (int)mCan_thread);
}

void Can_interface::can_close() {
  if (mCan_thread > 0) {
    void *recycle;
    /*回收线程资源  */
    pthread_join(mCan_thread, &recycle);
    pthread_mutex_destroy(&recive_lock);
    printf(">>: epoll thread destroy!\r\n");
  }
  /*关闭epoll*/
  if (mVcuCan.epoll_fd > 0) {
    close(mVcuCan.epoll_fd);
    printf(">>: epoll  close!\r\n");
  }
  /*关闭CAN外设*/
  if (mVcuCan.sock_fd > 0) {
    close(mVcuCan.sock_fd);
    printf(">>:  can socket thread destroy!\r\n");
  }
  printf(">>:  can device close!\r\n");
}

Can_interface::Can_interface() {}

Can_interface::~Can_interface() {}
