import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1) 
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton('192.168.1.1'))
while True:
    time.sleep(1)
    s.sendto(b"\x0B\x02\x2A\x00", ('224.0.1.129', 319))