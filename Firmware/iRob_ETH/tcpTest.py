# TCP communication to iRob ETH server test.

import sys
import time
import asyncio
import socket

class iRobETH:

    def __init__(self, ipAddr):
        if ipAddr is None:
            printf('iRobETH: Please provide iRob ETH IP address!')
            sys.exit()
    
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:# Try to connect to iRob ETH
            self.sock.connect((ipAddr, 6767))
            
        except:
            self.sock.close()
            print('iRobETH: Can\'t connect to the iRob ETH!')
            sys.exit()
        
        print('iRobETH: Connected to iRob ETH!')
        
        self.txPacket = list()

    def irobProtocolWrite(self, commandType, commandIndex, data):
        self.txPacket = ['R'.encode(), 'B'.encode()] # "RB" header
        length = 4 # Header + Command + data length
        
        # Add data length to the length byte if there is any
        if data is not None:
            length += len(data)
            
        command = (commandType << 5) | commandIndex | (1 << 7)  
        self.txPacket += list(command.to_bytes(1, byteorder='little'))
        self.txPacket += list(length.to_bytes(1, byteorder='little'))
        
        if data is not None:
            self.txPacket += data
        
        try:
            self.sock.send(bytes(self.txPacket))
            
        except socket.error():
            print('iRobETH: Error TX!')
    
    def irobProtocolRead(self, commandType, commandIndex, lengthRead):
        self.txPacket = [0x52, 0x42] # "RB" header
        length = 4 + lengthRead # Header + Command + data length
            
        command = (commandType << 5) | commandIndex
        self.txPacket += list(command.to_bytes(1, byteorder='little'))
        self.txPacket += list(length.to_bytes(1, byteorder='little'))

        try:
            self.sock.send(bytes(self.txPacket))
            
        except socket.error():
            print('iRobETH: Error TX!')

    def irobProtocolReturn(self, commandType, commandIndex, lengthRead):
        header = self.sock.recv(2)
        # if bytes(header) != bytes('JB'.encode()):
            # print('iRobETH: Header mismatch!')
            # return None
            
        cmd     = int.from_bytes(self.sock.recv(1), byteorder='little')
        length  = int.from_bytes(self.sock.recv(1), byteorder='little')
        
        print(f'header {header}')
        print(f'cmd {cmd}')
        print(f'length {length}')
        
        if(length < 1):
            print('iRobETH: Legth too short!')
            return None
        
        restData = self.sock.recv(length)
        
        print(restData)

    def irobReadParamStatus(self):
        self.irobProtocolRead(
            1,
            0,
            1
        )
    
        self.irobProtocolReturn(
            1,
            0,
            1
        )

def main():
    print('iRob ETH communication test code')
    irob_if = iRobETH('192.168.1.16')
    while(True):
        irob_if.irobReadParamStatus()

if __name__ == '__main__':
    main()