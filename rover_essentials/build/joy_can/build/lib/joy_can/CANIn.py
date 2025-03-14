#initial commnands to set up the CAN node
import os
import can
import time

import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Drive
from msg_interfaces.msg import ArmEndMotion
from msg_interfaces.msg import Encoder


#TODO modify the OS commands to set up CAN0 node only if it is not running 
#TODO polling command section execution
#TODO test with PWM and direction 
#TODO decide on conditions in order to execute systems check commands and polling flags
#TODO decide on ROS2 architecture to take care of the above point 
#TODO run a script to setup CAN networks using ifconfig at system start, errors were most likely due to reinitialization
#TODO develop logic to send commands to the CAN bus only when there are nodes connected to it

BIT_RATE=500000

NUM_NODES_CH0=6 #number of nodes on channel A
NUM_NODES_CH1=6 #number of nodes on channel B

SLAVE0_ID_CH0=0x550 #first node on channel 0
SLAVE0_ID_CH1=0X256 #first node on channel 1
MASTER_ID=0x265

#loop to generate all the slave node addresses
SLAVE_IDS0=list()
SLAVE_IDS1=list()

for i in range(NUM_NODES_CH0):
    SLAVE_IDS0.append(SLAVE0_ID_CH0+i)

for i in range(NUM_NODES_CH1):
    SLAVE_IDS1.append(SLAVE0_ID_CH1+i)
#SLAVE_IDS=[0x550,0x551,0x552,0x553,0x554]
                                                               
#lists to store if data has entered or not

#channel 0
upcheck0=[0]*NUM_NODES_CH0
datacheck0=[0]*NUM_NODES_CH0
CANCheck0=[0]*NUM_NODES_CH0
data0=[0]*NUM_NODES_CH0

#channel 1
upcheck1=[0]*NUM_NODES_CH1
datacheck1=[0]*NUM_NODES_CH1
CANCheck1=[0]*NUM_NODES_CH1
data1=[0]*NUM_NODES_CH1

DELAY=5000//(NUM_NODES_CH0+NUM_NODES_CH1)-50

#array to hold incoming 8 bytes of data for encoder from each node

#channel 0
encoderIn0=list()

for i in range(NUM_NODES_CH0):
    arr=[0]*8
    encoderIn0.append(arr)

#channel 1
encoderIn1=list()

for i in range(NUM_NODES_CH1):
    arr=[0]*8
    encoderIn1.append(arr)

#channel 0
diff0=[0]*NUM_NODES_CH0
pos0=[0]*NUM_NODES_CH0 #absolute position
angPos0=[0]*NUM_NODES_CH0 #angular position

#store PWM and direction channels for each motor
PWM0=[0]*NUM_NODES_CH0 #set 8 bit PWM value from 0 - 255
DIR0=[0]*NUM_NODES_CH0

#channel 1
diff1=[0]*NUM_NODES_CH1
pos1=[0]*NUM_NODES_CH1 #absolute position
angPos1=[0]*NUM_NODES_CH1 #angular position

#store PWM and direction channels for each motor
PWM1=[0]*NUM_NODES_CH1 #set 8 bit PWM value from 0 - 255
DIR1=[0]*NUM_NODES_CH1



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
        self.sysCANCheckFlag0=1 #flag to perform sysCANCheck
        self.pollingFlag0=0 #flag to start polling mode

        self.sysCANCheckFlag1=1 #flag to perform sysCANCheck
        self.pollingFlag1=0 #flag to start polling mode

        self.subscription_drive=self.create_subscription(Drive,'drive_commands',self.CAN_callback0,10)
        self.subscription_arm=self.create_subscription(ArmEndMotion,'arm_commands',self.CAN_callback1,10)

        self.publisher_encoder = self.create_publisher(Encoder,'encoder_values',10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def CAN_callback0(self,msg:Drive):
        #print("Received 0")
        #print(len(msg.direction))
        for i in range(len(msg.direction)):
            DIR0[i]=msg.direction[i] #update the values of PWM and DIR
            PWM0[i]=msg.speed[i]
        if msg.sys_check: #set flags accordingly 
            self.sysCANCheckFlag0=1
            self.pollingFlag0=0
        else:
            self.sysCANCheckFlag0=0
            self.pollingFlag0=1
    
    def CAN_callback1(self,msg:ArmEndMotion):    	
        #print("Received 1")        
        #print(len(msg.direction))
        for i in range(len(msg.direction)):
            DIR1[i]=msg.direction[i]
            PWM1[i]=msg.speed[i]
        if msg.sys_check: #set flags accordingly 
            self.sysCANCheckFlag1=1
            self.pollingFlag1=0
        else:
            self.sysCANCheckFlag1=0
            self.pollingFlag1=1




    def timer_callback(self):
        msg=Encoder()
        if self.sysCANCheckFlag0 or self.sysCANCheckFlag1:
            sysCANCheck(msg,can0,can1) #run systems check on all the CAN nodes
            print("Status channel 0:",upcheck0)
            print("Status channel 1:",upcheck1)
            time.sleep(0.2) #small delay before moving to next command
            #sysCANCheckFlag=0
        if self.pollingFlag0 or self.pollingFlag1:
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

def pending0(msg_rx:can.Message):
    #msg_rx=can.Message
    #create a loop for checking messages
    #print(f"Entered loop {msg_rx.data}")
    if msg_rx.dlc==1: #a systems check command
        for i in range(NUM_NODES_CH0):
            if msg_rx.arbitration_id == SLAVE_IDS0[i]:
                datacheck0[i]=1
                data0[i]=msg_rx.data[0]

    #use the above logic for up-link check of CAN nodes


    if msg_rx.dlc==8:
        for i in range(NUM_NODES_CH0):
            if msg_rx.arbitration_id==SLAVE_IDS0[i]:
                CANCheck0[i]=1
                for j in range(8):
                    encoderIn0[i][j]=msg_rx.data[j] #copy the bytes received
    #use the above logic for bytes reconstruction

def pending1(msg_rx:can.Message):
    #msg_rx=can.Message
    #create a loop for checking messages
    #print(f"Entered loop {msg_rx.data}")
    if msg_rx.dlc==1: #a systems check command
        for i in range(NUM_NODES_CH1):
            if msg_rx.arbitration_id == SLAVE_IDS1[i]:
                datacheck1[i]=1
                data1[i]=msg_rx.data[0]

    #use the above logic for up-link check of CAN nodes


    if msg_rx.dlc==8:
        for i in range(NUM_NODES_CH1):
            if msg_rx.arbitration_id==SLAVE_IDS1[i]:
                CANCheck1[i]=1
                for j in range(8):
                    encoderIn1[i][j]=msg_rx.data[j] #copy the bytes received
    #use the above logic for bytes reconstruction


def sysCANCheck(msg_tx:can.Message,bus0,bus1):
    msg_tx.dlc=1 #send 1 byte of data to the slave nodes for CAN system check
    query=[0]*8 #create a query buffer of 8 bytes to send to the CAN bus   
    
    #run systems check on channel 0    
    for i in range(NUM_NODES_CH0):
        upcheck0[i]=0 #reset the flag before checking
        address=SLAVE_IDS0[i]-SLAVE0_ID_CH0 #returns an 8 bit number
        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(sysCheck<<5) | address; #sysCheck command created

        query[0]=command
        msg_tx.data=query
        bus0.send(msg_tx)
        #print(f"Message sent {msg_tx.data}")


        time.sleep(5/1000.0);
        #wait 5 ms before checking for response

        if datacheck0[i]==1:
            #print(f"datacheck {i} = {datacheck[i]}")
            #mirroring back a 1 byte message, it should be same as command
            if data0[i]==command:
                upcheck0[i]=1 #this node is working correctly
                #print(f"Working {i}")

        datacheck0[i]=0
        data0[i]=0 #clean the data buffer
        time.sleep(1/1000.0) #wait 1 ms before testing the next node
        
    #run systems check on channel 1
    for j in range(len(query)):
        query[j]=0
    for i in range(NUM_NODES_CH1):
        upcheck1[i]=0 #reset the flag before checking
        address=SLAVE_IDS1[i]-SLAVE0_ID_CH1 #returns an 8 bit number
        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(sysCheck<<5) | address; #sysCheck command created

        query[0]=command
        msg_tx.data=query
        bus1.send(msg_tx)
        #print(f"Message sent {msg_tx.data}")


        time.sleep(5/1000.0);
        #wait 5 ms before checking for response

        if datacheck1[i]==1:
            #print(f"datacheck {i} = {datacheck[i]}")
            #mirroring back a 1 byte message, it should be same as command
            if data1[i]==command:
                upcheck1[i]=1 #this node is working correctly
                #print(f"Working {i}")

        datacheck1[i]=0
        data1[i]=0 #clean the data buffer
        time.sleep(1/1000.0) #wait 1 ms before testing the next node


def poll(msg_tx:can.Message,bus0,bus1):
    msg=Encoder()
    msg_tx.dlc=3 #send command of length 3
    #along with PWM and direction signals
    query=[0]*8 #query to transmit to central node
    
    #transmission for channel 0
    for i in range(NUM_NODES_CH0):
        #poll each CAN node and check if you receive a message back
        #a maximum of 32 nodes, which means last 5 bits to be used as address
        address=SLAVE_IDS0[i]-SLAVE0_ID_CH0; #returns an 8 bit number

        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(polling<<5) | address; #sysCheck command created

        query[0]=command; #0 for 550, 1 for 551, ...
        query[1]=PWM0[i]
        query[2]=DIR0[i]


        #send the CAN message
        msg_tx.data=query
        bus0.send(msg_tx)
         
        #wait 50 us before checking for message from slave node
        delay_us(50);
        if (CANCheck0[i]==1): #data received
            CANCheck0[i]=0; #reset the data flag

            pos0[i] = (encoderIn0[i][0]<<24) | (encoderIn0[i][1]<<16) | (encoderIn0[i][2]<<8) | (encoderIn0[i][3]);
            angPos0[i] = (encoderIn0[i][4]<<8) | (encoderIn0[i][5]);
            diff0[i] = (encoderIn0[i][6]<<8) | (encoderIn0[i][7]);

        print(f"MSG, channel 0, {i},{pos0[i]},{angPos0[i]},{diff0[i]}");

        delay_us(DELAY); #wait 1.25 ms =(5/4) ms before polling the next node (-50 for the previous delay of 20 us)
        #the polling interval is fixed here at 5 ms
    
    #polling for channel 1
    for j in range(len(query)):
        query[j]=0
    for i in range(NUM_NODES_CH1):
        #poll each CAN node and check if you receive a message back
        #a maximum of 32 nodes, which means last 5 bits to be used as address
        address=SLAVE_IDS1[i]-SLAVE0_ID_CH1; #returns an 8 bit number

        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(polling<<5) | address; #sysCheck command created

        query[0]=command; #0 for 550, 1 for 551, ...
        query[1]=PWM1[i]
        query[2]=DIR1[i]


        #send the CAN message
        msg_tx.data=query
        bus1.send(msg_tx)
         
        #wait 50 us before checking for message from slave node
        delay_us(50);
        if (CANCheck1[i]==1): #data received
            CANCheck1[i]=0; #reset the data flag

            pos1[i] = (encoderIn1[i][0]<<24) | (encoderIn1[i][1]<<16) | (encoderIn1[i][2]<<8) | (encoderIn1[i][3]);
            angPos1[i] = (encoderIn1[i][4]<<8) | (encoderIn1[i][5]);
            diff1[i] = (encoderIn1[i][6]<<8) | (encoderIn1[i][7]);

        print(f"MSG, channel 1, {i},{pos1[i]},{angPos1[i]},{diff1[i]}");

        delay_us(DELAY); #wait 1.25 ms =(5/4) ms before polling the next node (-50 for the previous delay of 20 us)
        #the polling interval is fixed here at 5 ms
    
    #converting the data into ROS2 topics
    msg.drive_node0 = [0,pos0[0],angPos0[0],diff0[0]]
    msg.drive_node1 = [1,pos0[1],angPos0[1],diff0[1]]
    msg.drive_node2 = [2,pos0[2],angPos0[2],diff0[2]]
    msg.drive_node3 = [3,pos0[3],angPos0[3],diff0[3]]
    msg.drive_node4 = [4,pos0[4],angPos0[4],diff0[4]]
    msg.drive_node5 = [5,pos0[5],angPos0[5],diff0[5]]
    msg.arm_node0 = [0,pos1[0],angPos1[0],diff1[0]]
    msg.arm_node1 = [1,pos1[1],angPos1[1],diff1[1]]
    msg.arm_node2 = [2,pos1[2],angPos1[2],diff1[2]]
    msg.arm_node3 = [3,pos1[3],angPos1[3],diff1[3]]
    msg.arm_node4 = [4,pos1[4],angPos1[4],diff1[4]]
    msg.arm_node5 = [5,pos1[5],angPos1[5],diff1[5]]
    return msg
    
#TODO run the CAN.sh script before this


can0 = can.interface.Bus(channel = 'can0', interface = 'socketcan',receive_own_messages=True) 
can1 = can.interface.Bus(channel = 'can1', interface = 'socketcan',receive_own_messages=True)
#prevent buffer overflow if one channel is disconnected

#printer=can.Printer()
#listener = msg_rx_routine                                             
#listener = pending                                         
notifier0 = can.Notifier(can0, [pending0])  #assign listener to notifier
notifier1 = can.Notifier(can1, [pending1])

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





