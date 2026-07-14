#ifndef APP_TCPSERVER_H
#define APP_TCPSERVER_H

#include "systick.h"
#include "eth_driver.h"
#include "net_config.h"

#define ROS_TCP_PORT	6767

#ifdef UDP_PORT
#define SIDE_UDP_PORT	0
#endif

extern uint8_t u8SocketRecvBuf[RECE_BUF_LEN];

void vAppTcp_serverInit(uint8_t u8IpNum);
uint8_t vAppTcp_createRosListenSocket(void);
uint8_t u8AppTcp_isRosClientConnected(void);
void vAppTcp_handleIpInterrupt(void);

#endif