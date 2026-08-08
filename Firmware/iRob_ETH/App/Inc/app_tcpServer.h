#ifndef APP_TCPSERVER_H
#define APP_TCPSERVER_H

#include "systick.h"
#include "eth_driver.h"
#include "net_config.h"

#define ROS_TCP_PORT	6767
#define UDP_PORT

#ifdef UDP_PORT
#define PTP_EVENT_PORT    319
#define PTP_GENERAL_PORT  320
#endif

#define TCP_SOCKET_ID	3

extern uint8_t u8SocketRecvBuf[RECE_BUF_LEN];
extern uint8_t u8SocketSendBuf[255];

void vAppTcp_serverInit(uint8_t u8IpNum);
uint8_t vAppTcp_createRosListenSocket(void);
#ifdef UDP_PORT
uint8_t vAppTcp_createUdpPTPPort(void);
#endif
uint8_t u8AppTcp_isRosClientConnected(void);
uint8_t u8AppTcp_isThereDataToRead(void);
void vAppTcp_handleIpInterrupt(void);

#endif