#initial commnands to set up the CAN node
import os
import can
import time

import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Drive
from msg_interfaces.msg import EncoderDrive


#TODO modify the OS commands to set up CAN0 node only if it is not running 
#TODO polling command section execution
#TODO test with PWM and direction 
#TODO decide on conditions in order to execute systems check commands and polling flags
#TODO decide on ROS2 architecture to take care of the above point 
#TODO run a script to setup CAN networks using ifconfig at system start, errors were most likely due to reinitialization
#TODO develop logic to send commands to the CAN bus only when there are nodes connected to it

BIT_RATE=500000

NUM_NODES_CH0=6 #number of nodes on channel A

SLAVE0_ID_CH0=0x550 #first node on channel 0
MASTER_ID=0x465

#loop to generate all the slave node addresses
SLAVE_IDS0=list()
SLAVE_IDS1=list()

for i in range(NUM_NODES_CH0):
    SLAVE_IDS0.append(SLAVE0_ID_CH0+i)

#SLAVE_IDS=[0x550,0x551,0x552,0x553,0x554]
                                                               
#lists to store if data has entered or not

#channel 0
upcheck0=[0]*NUM_NODES_CH0
datacheck0=[0]*NUM_NODES_CH0
CANCheck0=[0]*NUM_NODES_CH0
data0=[0]*NUM_NODES_CH0


DELAY=5000//NUM_NODES_CH0-50

#array to hold incoming 8 bytes of data for encoder from each node

#channel 0
encoderIn0=list()

for i in range(NUM_NODES_CH0):
    arr=[0]*8
    encoderIn0.append(arr)

#channel 0
diff0=[0]*NUM_NODES_CH0
pos0=[0]*NUM_NODES_CH0 #absolute position
angPos0=[0]*NUM_NODES_CH0 #angular position

#store PWM and direction channels for each motor
PWM0=[0]*NUM_NODES_CH0 #set 8 bit PWM value from 0 - 255
DIR0=[0]*NUM_NODES_CH0


#PWM[0]=20 #for testing, comment out later 
#DIR[0]=1

#PWM[1]=50
#DIR[1]=0

sysCheck=0 #000 -> 0
polling=7 #111->7  (start polling)
stopPoll=5 #101->7 (stop polling)
reset=1 #001 -> 7 (reset encoders)

class CAN_Publisher(Node):
    
    def __init__(self):
        super().__init__('Drive_Master')
        #update rate of main loop 
        timer_period = 0.01
        self.sysCANCheckFlag0=1 #flag to perform sysCANCheck
        self.pollingFlag0=0 #flag to start polling mode
        self.resetMode=0 #flag to reset the encoders

        self.subscription_drive=self.create_subscription(Drive,'/drive_commands',self.CAN_callback0,10)
        self.publisher_encoder = self.create_publisher(EncoderDrive,'encoder_drive',10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def CAN_callback0(self,msg:Drive):
        #print("Received 0")
        #print(len(msg.direction))
        for i in range(len(msg.direction)):
	    #code to account for change in motor direction during manufacturing
            if i<=2:
                DIR0[i] = abs(1-msg.direction[i])
            else:
                DIR0[i]=msg.direction[i]
            #DIR0[i]=msg.direction[i] #update the values of PWM and DIR
            PWM0[i]=msg.speed[i]
        if msg.reset:
            self.resetMode=1
            self.sysCANCheckFlag0=0
            self.pollingFlag0=0
        else:
            self.resetMode=0
            if msg.sys_check: #set flags accordingly 
                self.sysCANCheckFlag0=1
                self.pollingFlag0=0
            else:
                self.sysCANCheckFlag0=0
                self.pollingFlag0=1
        
    
    def timer_callback(self):
        if self.resetMode:
            resetEnc(msg,can0)
        if self.sysCANCheckFlag0:
            sysCANCheck(msg,can0) #run systems check on all the CAN nodes
            out  = f"Status channel 0: {upcheck0}"
            self.get_logger().info(out)
            time.sleep(0.2) #small delay before moving to next command
            #sysCANCheckFlag=0
        if self.pollingFlag0:
            enc_msg=poll(msg,can0)
            self.publisher_encoder.publish(enc_msg)
            #self.get_logger().info(list(enc_msg.drive_node0))
            #self.get_logger().info(list(enc_msg.drive_node1))
            #self.get_logger().info(list(enc_msg.drive_node2))
            #self.get_logger().info(list(enc_msg.drive_node3))
            #self.get_logger().info(list(enc_msg.drive_node4))
            #self.get_logger().info(list(enc_msg.drive_node5)
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

def resetEnc(msg_tx:can.Message,bus0):
    msg_tx.dlc=1 #send 1 byte of data for encoder reset
    query = [0]*8
    #run systems check on channel 0    
    for i in range(NUM_NODES_CH0):
        address=SLAVE_IDS0[i]-SLAVE0_ID_CH0 #returns an 8 bit number
        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(reset<<5) | address; #sysCheck command created

        query[0]=command
        msg_tx.data=query
        bus0.send(msg_tx)
        #print(f"Message sent {msg_tx.data}")
        delay_us(10); #small delay before engaging the bus 



def sysCANCheck(msg_tx:can.Message,bus0):
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

def poll(msg_tx:can.Message,bus0):
    enc_msg=EncoderDrive()
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
    
    #converting the data into ROS2 topics
    enc_msg.drive_node0 = [0,pos0[0],angPos0[0],diff0[0]]
    enc_msg.drive_node1 = [1,pos0[1],angPos0[1],diff0[1]]
    enc_msg.drive_node2 = [2,pos0[2],angPos0[2],diff0[2]]
    enc_msg.drive_node3 = [3,pos0[3],angPos0[3],diff0[3]]
    enc_msg.drive_node4 = [4,pos0[4],angPos0[4],diff0[4]]
    enc_msg.drive_node5 = [5,pos0[5],angPos0[5],diff0[5]]
    return enc_msg
    
#TODO run the CAN.sh script before this


can0 = can.interface.Bus(channel = 'can0', interface = 'socketcan',receive_own_messages=False) 
#prevent buffer overflow if one channel is disconnected

#printer=can.Printer()
#listener = msg_rx_routine                                             
#listener = pending                                         
notifier0 = can.Notifier(can0, [pending0])  #assign listener to notifier

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





