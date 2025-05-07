#initial commnands to set up the CAN node
#import os
import can
import time

import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Drive
from msg_interfaces.msg import ArmEndMotion
from msg_interfaces.msg import Encoder

#TODO polling command section execution
#TODO test with PWM and direction 
#TODO decide on conditions in order to execute systems check commands and polling flags
#TODO decide on ROS2 architecture to take care of the above point 
#TODO run a script to setup CAN networks using ifconfig at system start, errors were most likely due to reinitialization
#TODO develop logic to send commands to the CAN bus only when there are nodes connected to it

BIT_RATE=500000

NUM_NODES = [6, 6] #number of nodes on channel A, B respectively

SLAVE_IDS_first = [0x550, 0X256] #first node on channel 0, 1 respectively

MASTER_ID=0x265

#all the slave node addresses
SLAVE_IDS = [list(range(SLAVE_IDS_first[0], SLAVE_IDS_first[0] + NUM_NODES[0])),
             list(range(SLAVE_IDS_first[1], SLAVE_IDS_first[1] + NUM_NODES[1]))]

#SLAVE_IDS=[0x550,0x551,0x552,0x553,0x554]

#lists to store if data has entered or not

upcheck = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]
datacheck = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]
CANCheck = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]
data = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]

DELAY=5000//(NUM_NODES[0]+NUM_NODES[1])-50

#array to hold incoming 8 bytes of data for encoder from each node

#channel 0
encoderIn = []

for i in [0,1]:
    for i in range(NUM_NODES[i]):
        encoderIn[i].append([0]*8)

#channel 0
diff = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]
pos = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]] # absolute position
angPos = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]] # angular position

# store PWM and direction channels for each motor
PWM = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]] # set 8-bit PWM value from 0 - 255
DIR = [[0]*NUM_NODES[0], [0]*NUM_NODES[1]]


#PWM[0]=20 #for testing, comment out later 
#DIR[0]=1

#PWM[1]=50
#DIR[1]=0

sysCheck=0 #000 -> 0
polling=7 #111->7  (start polling)
stopPoll=5 #101->7 (stop polling)


class CAN_Publisher(Node):
    
    def __init__(self):
        super().__init__('CAN_Master')
        #update rate of main loop 
        timer_period = 0.01

        self.sysCANCheckFlag = [1, 1] #flag to perform sysCANCheck
        self.pollingFlag = [0,0] #flag to start polling mode

        self.subscription_drive = self.create_subscription(Drive, 'drive_commands', lambda msg: self.CAN_callback(msg, channel=0), 10)
        self.subscription_arm = self.create_subscription(ArmEndMotion, 'arm_commands', lambda msg: self.CAN_callback(msg, channel=1), 10)

        self.publisher_encoder = self.create_publisher(Encoder,'encoder_values',10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def CAN_callback(self, msg:Drive, channel):
        #print("Received 0")
        #print(len(msg.direction))
        for i in range(len(msg.direction)):
            DIR[channel][i]=msg.direction[i] #update the values of PWM and DIR
            PWM[channel][i]=msg.speed[i]
        if msg.sys_check: #set flags accordingly 
            self.sysCANCheckFlag[channel] = 1
            self.pollingFlag[channel] = 0
        else:
            self.sysCANCheckFlag[channel] = 0
            self.pollingFlag[channel] = 1
        
    def timer_callback(self):
        msg=Encoder()
        if self.sysCANCheckFlag[0] or self.sysCANCheckFlag[1]:
            sysCANCheck(msg, can0, can1) #run systems check on all the CAN nodes
            print("Status channel 0:",upcheck[0])
            print("Status channel 1:",upcheck[1])
            time.sleep(0.2) #small delay before moving to next command
            #sysCANCheckFlag=0
        if self.pollingFlag[0] or self.pollingFlag:
            msg=poll(msg,can0,can1)
            self.publisher_encoder.publish(msg)
            #if i<2000:
            #    #poll(msg,can0)
            #    i+=1
            #else:
            #    PWM[0]=0
            #    PWM[1]=0
            
        delay_us(20)
              



def delay_us(micros):
    time.sleep(micros/1000000.0)

def pending(msg_rx:can.Message, channel):
    #msg_rx=can.Message
    #create a loop for checking messages
    #print(f"Entered loop {msg_rx.data}")
    if msg_rx.dlc==1: #a systems check command
        for i in range(NUM_NODES[channel]):
            if msg_rx.arbitration_id == SLAVE_IDS[channel][i]:
                datacheck[channel][i]=1
                data[channel][i]=msg_rx.data[0]

    if msg_rx.dlc==8:
        for i in range(NUM_NODES[channel]):
            if msg_rx.arbitration_id==SLAVE_IDS[channel][i]:
                CANCheck[channel][i]=1
                for j in range(8):
                    encoderIn[channel][i][j]=msg_rx.data[j] #copy the bytes received
    #use the above logic for bytes reconstruction

def sysCANCheck(msg_tx:can.Message,bus0,bus1):
    msg_tx.dlc=1 #send 1 byte of data to the slave nodes for CAN system check
    
    #run systems check on channel 0
    for channel in [0,1]:
        query=[0]*8 #create a query buffer of 8 bytes to send to the CAN bus
        for i in range(NUM_NODES[channel]):
            upcheck[channel][i]=0 #reset the flag before checking
            address=SLAVE_IDS[channel][i]-SLAVE_IDS_first[channel] #returns an 8 bit number
            #as there are only 32 nodes maximum, the last 5 bits will be filled

            command=(sysCheck<<5) | address; #sysCheck command created

            query[0]=command
            msg_tx.data=query
            
            if channel == 0: 
                bus0.send(msg_tx)
            else:
                bus1.send(msg_tx)
            #print(f"Message sent {msg_tx.data}")


            time.sleep(5/1000.0);
            #wait 5 ms before checking for response

            if datacheck[channel][i]==1:
                #print(f"datacheck {i} = {datacheck[i]}")
                #mirroring back a 1 byte message, it should be same as command
                if data[channel][i]==command:
                    upcheck[channel][i]=1 #this node is working correctly
                    #print(f"Working {i}")

            datacheck[channel][i]=0
            data[channel][i]=0 #clean the data buffer
            time.sleep(1/1000.0) #wait 1 ms before testing the next node

def poll(msg_tx:can.Message,bus0,bus1):
    msg=Encoder()
    msg_tx.dlc=3 #send command of length 3
    #along with PWM and direction signals
    for channel in [0, 1]:
        query=[0]*8 #query to transmit to central node
        
        #transmission for channel 0
        for i in range(NUM_NODES[channel]):
            #poll each CAN node and check if you receive a message back
            #a maximum of 32 nodes, which means last 5 bits to be used as address
            address=SLAVE_IDS[channel][i]-SLAVE_IDS_first[channel]; #returns an 8 bit number

            #as there are only 32 nodes maximum, the last 5 bits will be filled

            command=(polling<<5) | address; #sysCheck command created

            query[0]=command; #0 for 550, 1 for 551, ...
            query[1]=PWM[channel][i]
            query[2]=DIR[channel][i]


            #send the CAN message
            msg_tx.data=query
            if channel == 0:
                bus0.send(msg_tx)
            else:
                bus1.send(msg_tx)
            
            #wait 50 us before checking for message from slave node
            delay_us(50);
            if (CANCheck[channel][i]==1): #data received
                CANCheck[channel][i]=0; #reset the data flag

                pos[channel][i] = (encoderIn[channel][i][0]<<24) | (encoderIn[channel][i][1]<<16) | (encoderIn[channel][i][2]<<8) | (encoderIn[channel][i][3]);
                angPos[channel][i] = (encoderIn[channel][i][4]<<8) | (encoderIn[channel][i][5]);
                diff[channel][i] = (encoderIn[channel][i][6]<<8) | (encoderIn[channel][i][7]);

            print(f"MSG, channel {channel}, {i},{pos[channel][i]},{angPos[channel][i]},{diff[channel][i]}");

            delay_us(DELAY); #wait 1.25 ms =(5/4) ms before polling the next node (-50 for the previous delay of 20 us)
            #the polling interval is fixed here at 5 ms
            
    #converting the data into ROS2 topics
    msg.drive_node0 = [0,pos[0][0],angPos[0][0],diff[0][0]]
    msg.drive_node1 = [1,pos[0][1],angPos[0][1],diff[0][1]]
    msg.drive_node2 = [2,pos[0][2],angPos[0][2],diff[0][2]]
    msg.drive_node3 = [3,pos[0][3],angPos[0][3],diff[0][3]]
    msg.drive_node4 = [4,pos[0][4],angPos[0][4],diff[0][4]]
    msg.drive_node5 = [5,pos[0][5],angPos[0][5],diff[0][5]]
    msg.arm_node0 = [0,pos[1][0],angPos[1][0],diff[1][0]]
    msg.arm_node1 = [1,pos[1][1],angPos[1][1],diff[1][1]]
    msg.arm_node2 = [2,pos[1][2],angPos[1][2],diff[1][2]]
    msg.arm_node3 = [3,pos[1][3],angPos[1][3],diff[1][3]]
    msg.arm_node4 = [4,pos[1][4],angPos[1][4],diff[1][4]]
    msg.arm_node5 = [5,pos[1][5],angPos[1][5],diff[1][5]]
    return msg
    
#TODO run the CAN.sh script before this


can0 = can.interface.Bus(channel = 'can0', interface = 'socketcan',receive_own_messages=True) 
can1 = can.interface.Bus(channel = 'can1', interface = 'socketcan',receive_own_messages=True)

#prevent buffer overflow if one channel is disconnected

#printer=can.Printer()
#listener = msg_rx_routine                                             
#listener = pending            
                             
notifier0 = can.Notifier(can0, [lambda msg: pending(msg, channel=0)])  # assign listener to notifier
notifier1 = can.Notifier(can1, [lambda msg: pending(msg, channel=1)])

#can.Notifier(can0, [pending(can)])
#msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7])

msg = can.Message(arbitration_id=MASTER_ID, is_extended_id=False, is_remote_frame=False)

def main(args=None):
    rclpy.init(args=args)
    master=CAN_Publisher()
        
    try:
        #os.system(f'sudo ip link set can0 type can bitrate {BIT_RATE}')
        #os.system('sudo ifconfig can0 up')
        #os.system('sudo ifconfig can0 txqueuelen 65536')
        rclpy.spin(master)            
    except KeyboardInterrupt:
        master.destroy_node() #kill node on interruption
        rclpy.shutdown()   
        #os.system('sudo ifconfig can0 down') #shutdown CAN0 network    
    except Exception as e:
        print(e)
    
if __name__=='main':
    main()
