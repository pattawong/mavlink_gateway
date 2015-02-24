'''
    PC Ground Station for MAVLink Gateway
    modified from Simple udp socket server
    by Silver Moon (m00n.silv3r@gmail.com)
'''
 
import socket
import sys,struct
 
HOST = '192.168.1.43'   # Symbolic name meaning all available interfaces
PORT = 5000 # Arbitrary non-privileged port
 
# Datagram (udp) socket
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print 'Socket created'
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
 
 
# Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
print 'Socket bind complete'
 
#now keep talking with the client
while 1:
    # receive data from client (data, addr)
    d = s.recvfrom(1024)
    data = d[0]
    addr = d[1]
     
    if not data: 
        break
     
    reply = "OK"

    recv_data = data.strip()
##    print 'Message[' + addr[0] + ':' + str(addr[1]) + '] - ' + recv_data
    if ord(recv_data[0]) == 0xFE:
##        print "stx"
        #MSG ID 11
        if ord(recv_data[1]) == 11:
            mav_mode = recv_data[2:11]
            mav_type = ord(recv_data[11])
            mav_arm = ord(recv_data[12])
            print "STATUS : "+mav_mode + " TYPE : "+str(mav_type) + " ARMED : "+str(mav_arm)
        #MSG ID 13
        if ord(recv_data[1]) == 13:
            try:
                tagData = recv_data[2:6][::-1]
                temp1 = struct.unpack("f", tagData)
                tagData = recv_data[6:10][::-1]
                temp2 = struct.unpack("f", tagData)
                tagData = recv_data[10:14][::-1]
                temp3 =  struct.unpack("f", tagData)
            except:
                print 'error'
            print "ATITTUDE : "+str(temp1[0])+","+str(temp2[0])+","+str(temp3[0])
     
s.close()
