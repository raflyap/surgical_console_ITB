#!/usr/bin/env python

import socket
import time
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('192.168.123.2', 1920)
message = ';95.9677;126.5344;75.0000;-0.0000;0.0000;-0.0000;13.5;0;1;1'
message = message.encode('utf-8')
print('sending "%s"' % message)

sum = 0
for x in range (5) :

    # Send data
    sent = sock.sendto(message, server_address)
    start_time = time.time()

    # Receive response
    data, address = sock.recvfrom(4096)
    stop_time = time.time()
    print(x+1)
    rtt = ((stop_time - start_time)/2) * 1000
    print('Start : %.4f\tEnd : %.4f' % (start_time,stop_time))
    print('Latency : %.4f\n' % rtt)
    sum += rtt

print('\n\nAverage : %.4f' % (sum/5))
sock.close()