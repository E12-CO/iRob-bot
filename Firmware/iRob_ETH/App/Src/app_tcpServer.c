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
SOCK_INF	tUDPSocketInfo;
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
	
	// Socket connected
    if (u8SocketStatus & SINT_STAT_CONNECT){
		// Only register a TCP socket that is for data exchange
		if(u8SocketId == 1){
			u8IsRosClientConnected = 1;
			WCHNET_ModifyRecvBuf(
				u8SocketId, 
				(uint32_t)&u8SocketRecvBuf,// Pass the Address, not pointer
				RECE_BUF_LEN);
		}
		
    }
	
	// Socket disconnect or timed out
    if (u8SocketStatus & (SINT_STAT_DISCONNECT | SINT_STAT_TIM_OUT)){
		if(u8SocketId == 1)
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
		
		// Handle the Listening socket
		u8SocketIntStatus = WCHNET_GetSocketInt(u8SocketIdForListen);
		if (u8SocketIntStatus)
			vAppTcp_handleSocketInterrupt(u8SocketIdForListen, u8SocketIntStatus);
		
		// Handle the actual TCP connection
		u8SocketIntStatus = WCHNET_GetSocketInt(1);
		if (u8SocketIntStatus)
			vAppTcp_handleSocketInterrupt(1, u8SocketIntStatus);

    }
}
