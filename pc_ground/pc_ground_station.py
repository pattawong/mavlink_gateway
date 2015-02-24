'''
    PC Ground Station for MAVLink Gateway
    modified from Simple udp socket server
    by Silver Moon (m00n.silv3r@gmail.com)
'''
 
import socket
import sys,struct
 
HOST = '192.168.1.44'   # Symbolic name meaning all available interfaces
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
        print "MSG ID "+str(ord(recv_data[1]))
        #MSG ID 11
        if ord(recv_data[1]) == 11:
            mav_mode = recv_data[2:11]
            mav_type = ord(recv_data[11])
            mav_arm = ord(recv_data[12])
            print "MODE : "+mav_mode + " TYPE : "+str(mav_type) + " ARMED : "+str(mav_arm) 

        #MSG ID 12
        if ord(recv_data[1]) == 12:
            mav_fix_type = ord(recv_data[2])
            mav_hdop = ord(recv_data[3])*256 + ord(recv_data[4])
            mav_num_sat = ord(recv_data[5])
            mav_volt_bat = ord(recv_data[6])*256 + ord(recv_data[7])
            print "GPS FIX TYPE : "+ str(mav_fix_type) +" HDOP : "+ str(mav_hdop) + " Num Sat. : " + str(mav_num_sat)+ " Voltage : "+ str(mav_volt_bat)
     
        #MSG ID 13
        if ord(recv_data[1]) == 13:
            try:
                tagData = recv_data[2:6][::-1]
                roll = struct.unpack("f", tagData)
                tagData = recv_data[6:10][::-1]
                pitch = struct.unpack("f", tagData)
                tagData = recv_data[10:14][::-1]
                yaw =  struct.unpack("f", tagData)
            except:
                print 'error'
            print "ATITTUDE : "+str(roll[0])+","+str(pitch[0])+","+str(yaw[0])

        #MSG ID 14
        if ord(recv_data[1]) == 14:
            try:
                tagData = recv_data[2:6][::-1]
                alt = struct.unpack("f", tagData)
                heading = ord(recv_data[6])*256 + ord(recv_data[7])
                tagData = recv_data[8:12][::-1]
                ground_speed = struct.unpack("f", tagData)
            except:
                print 'error'

            print "ALT : "+str(alt[0])+" HEADING : "+str(heading)+" GROUND SPEED : "+str(ground_speed[0])
        #MSG ID 15
        if ord(recv_data[1]) == 15:
            raw_ch = []
            for i in range(8): 
                raw_ch.append(ord(recv_data[2+i*2])*256 + ord(recv_data[3+i*2])) 

            print "RAW CH : " + str(raw_ch)
     
s.close()
