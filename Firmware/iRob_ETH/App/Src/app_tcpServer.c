#include "app_tcpServer.h"

uint8_t u8DeviceMACAddr[6]; //MAC address
uint8_t u8DeviceIpAddr[4] = {192, 168, 1, 0};                       //IP address
uint8_t u8GateWayIpAddr[4] = {192, 168, 1, 1};                      //Gateway IP address
uint8_t u8SubnetMask[4] = {255, 255, 255, 0};                      //subnet mask

uint8_t u8SocketIdForListen;            // Socket for Listening
uint8_t u8SocketRecvBuf[RECE_BUF_LEN];	// Socket receive buffer
uint8_t u8SocketSendBuf[255];			// Socket send buffer
uint32_t u32TcpRecvLen;
uint8_t u8IsRosClientConnected = 0;
uint8_t u8HasDataToRead = 0;

// Sockets
SOCK_INF	tRosSocketInfo;

#ifdef UDP_PORT
const uint8_t u8PtpMulticastIp[4] = {224, 0, 1, 129};
SOCK_INF	tUDPSocketInfo[2];

uint8_t u8PTPSocketEvent;			// Socket ID for PTP Event
uint8_t u8PTPSocketGeneral;			// Socket ID for PTP General
uint8_t u8UDPRecvBuf[10];
#endif

// Ethernet IP Stack handler
void ETH_IRQHandler(void){
    WCHNET_ETHIsr();
}


void vAppTcp_serverInit(uint8_t u8IpNum){

	// Assign the IP address sampled from the Dial rotary switch
	u8DeviceIpAddr[3] = u8IpNum + 16;
	
	// Get Mac Address 
	WCHNET_GetMacAddr(u8DeviceMACAddr);
	
	// Initialize Ethernet Library with MAC and IP stuff
	ETH_LibInit(
		u8DeviceIpAddr, 
		u8GateWayIpAddr, 
		u8SubnetMask, 
		u8DeviceMACAddr);
	
	// Enable keep alive
	struct _KEEP_CFG cfg;

	cfg.KLIdle = 20000;
	cfg.KLIntvl = 15000;
	cfg.KLCount = 9;
	WCHNET_ConfigKeepLive(&cfg);
}

// Setup the TCP listening socket for incoming ROS communication
uint8_t vAppTcp_createRosListenSocket(void){
	
	// Initialize the socket information for ROS comm TCP port
    tRosSocketInfo.SourPort = ROS_TCP_PORT;
    tRosSocketInfo.ProtoType = PROTO_TYPE_TCP;
	tRosSocketInfo.AppCallBack	= 0;
    if(
		WCHNET_SocketCreat(&u8SocketIdForListen, &tRosSocketInfo) != 
		WCHNET_ERR_SUCCESS
	){// Return 1 on the failed socket creation
		return 1;
	}else if(
		WCHNET_SocketListen(u8SocketIdForListen) != 
		WCHNET_ERR_SUCCESS
	){// Return 2 on the failed socket listening
		return 2;
	}            
		
	return 0;
}

#ifdef UDP_PORT
// UDP receive callback
void vAppTcp_udpReceiver(
	struct _SOCK_INF *socinf, 
	uint32_t ipaddr, 
	uint16_t port, 
	uint8_t *buf, 
	uint32_t len){
	
	u8 ip_addr[4], i;

    for (i = 0; i < 4; i++) {
        ip_addr[i] = ipaddr & 0xff;
        ipaddr = ipaddr >> 8;
    }	
		
	// Reply back 
	WCHNET_SocketUdpSendTo(
		socinf->SockIndex, 
		buf, 
		&len, 
		ip_addr, 
		port);
}

// Setup the UDP multicast connection for PTP
uint8_t vAppTcp_createUdpPTPPort(void){
	
	// Initialize the socket information for UDP
	tUDPSocketInfo[0].SourPort		= PTP_EVENT_PORT;
	tUDPSocketInfo[0].ProtoType 	= PROTO_TYPE_UDP;
	tUDPSocketInfo[0].IPAddr[0]		= u8PtpMulticastIp[0];
	tUDPSocketInfo[0].IPAddr[1]		= u8PtpMulticastIp[1];
	tUDPSocketInfo[0].IPAddr[2]		= u8PtpMulticastIp[2];
	tUDPSocketInfo[0].IPAddr[3]		= u8PtpMulticastIp[3];
	tUDPSocketInfo[0].RecvStartPoint = (uint32_t)u8UDPRecvBuf;
	tUDPSocketInfo[0].RecvBufLen	= 10;
	tUDPSocketInfo[0].AppCallBack	= vAppTcp_udpReceiver;
	if(
		WCHNET_SocketCreat(&u8PTPSocketEvent, &tUDPSocketInfo[0]) != 
		WCHNET_ERR_SUCCESS
	){// Return 1 on the failed socket creation
		return 1;
	} 
	
	tUDPSocketInfo[1].SourPort		= PTP_GENERAL_PORT;
	tUDPSocketInfo[1].ProtoType 	= PROTO_TYPE_UDP;
	tUDPSocketInfo[1].IPAddr[0]		= u8PtpMulticastIp[0];
	tUDPSocketInfo[1].IPAddr[1]		= u8PtpMulticastIp[1];
	tUDPSocketInfo[1].IPAddr[2]		= u8PtpMulticastIp[2];
	tUDPSocketInfo[1].IPAddr[3]		= u8PtpMulticastIp[3];
	tUDPSocketInfo[1].RecvStartPoint = (uint32_t)u8UDPRecvBuf;
	tUDPSocketInfo[1].RecvBufLen	= 10;
	tUDPSocketInfo[1].AppCallBack	= vAppTcp_udpReceiver;
	if(
		WCHNET_SocketCreat(&u8PTPSocketGeneral, &tUDPSocketInfo[1]) != 
		WCHNET_ERR_SUCCESS
	){// Return 2 on the failed socket creation
		return 2;
	} 


	return 0;
}
#endif 

uint8_t u8AppTcp_isRosClientConnected(void){
	return u8IsRosClientConnected;
}

uint8_t u8AppTcp_isThereDataToRead(void){
	if(u8HasDataToRead == 0)
		return 0;
	
	u8HasDataToRead = 0;
	return 1;
}

void vAppTcp_handleSocketInterrupt(
	uint8_t u8SocketId,
	uint8_t u8SocketStatus){
	
	// Recived data packet, run the callback function
    if (u8SocketStatus & SINT_STAT_RECV){
		
		// Handle TCP receiver
		if(u8SocketId == TCP_SOCKET_ID){
			if( u32TcpRecvLen = WCHNET_SocketRecvLen(u8SocketId, NULL),
				u32TcpRecvLen > 0){
				WCHNET_SocketRecv(
					u8SocketId,
					NULL,
					&u32TcpRecvLen
				);
				u8HasDataToRead = 1;
				// Update the pointer to point it back at the beginning 
				WCHNET_ModifyRecvBuf(
					u8SocketId, 
					(uint32_t)&u8SocketRecvBuf,// Pass the Address, not pointer
					RECE_BUF_LEN);
			}
		}	
    }
	
	// Socket connected
    if (u8SocketStatus & SINT_STAT_CONNECT){
		// Handle a single TCP connection
		if(u8SocketId == TCP_SOCKET_ID){
			u8IsRosClientConnected = 1;
			
			WCHNET_SocketSetKeepLive(u8SocketId, ENABLE);
			
			WCHNET_ModifyRecvBuf(
				u8SocketId, 
				(uint32_t)&u8SocketRecvBuf,// Pass the Address, not pointer
				RECE_BUF_LEN);
		}
		
    }
	
	// Socket disconnect or timed out
    if (u8SocketStatus & (SINT_STAT_DISCONNECT | SINT_STAT_TIM_OUT)){
		if(u8SocketId == 3)
			u8IsRosClientConnected = 0;
    }

}

// Handle the LWIP interrupts
void vAppTcp_handleIpInterrupt(void){
	u8 intstat;
    u8 u8SocketIntStatus;

    intstat = WCHNET_GetGlobalInt(); //get global interrupt flag
    if (intstat & GINT_STAT_UNREACH) //Unreachable interrupt
		__NOP();
	
    if (intstat & GINT_STAT_IP_CONFLI) //IP conflict
        __NOP();
	
    if (intstat & GINT_STAT_PHY_CHANGE){ //PHY status change
        if (WCHNET_GetPHYStatus() & PHY_Linked_Status)
			__NOP();
    }
	
    if (intstat & GINT_STAT_SOCKET){ //socket related interrupt
		// Handle each socket individually
		for(uint8_t idx=0; idx < WCHNET_MAX_SOCKET_NUM; idx++){
			u8SocketIntStatus = WCHNET_GetSocketInt(idx);
			if (u8SocketIntStatus)
				vAppTcp_handleSocketInterrupt(idx, u8SocketIntStatus);
		}
    }
}
