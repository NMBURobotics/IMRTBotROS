# NMBU Robotics 2019
# Author: Lars Grimstad
# A simple module for communicating with the NMBU Robotics IMRT100 robots
# through serial.


import sys
import serial
import threading
import time
import signal
import ctypes
from math import pi



# Class for communicating with the NMBU IMRT100 robot
class IMRTRobotSerial :

    MSG_SIZE = 26
    SPEED_MULTIPLIER = 100

    # Constructor
    def __init__(self):

        print'{}: NMBU Robotics imrt100 motor serial'.format(__name__)

        # Mutex
        self._mutex = threading.Lock()

        # Robot paramters
        self._wheel_radius = 0.075

        # Sonic members
        self._dist_1 = 255
        self._dist_2 = 255
        self._dist_3 = 255
        self._dist_4 = 255
        self._v1 = 0
        self._v2 = 0

        # Motor feedback members
        self._motor_1_current = 0
        self._motor_1_speed = 0
        self._motor_2_current = 0
        self._motor_2_speed = 0
        
        # Create an event for signaling threads when its time terminate the program
        self._run_event = threading.Event()
        self._run_event.set()

        self.shutdown_now = False
        #signal.signal(signal.SIGINT, self._shutdown_signal)
        

        # Thread for receiving data through serial
        self._rx_thread_ = threading.Thread(target=self._rx_thread)



        
    # Method for opening serial port
    def connect(self, port_name='/dev/ttyACM0'):
        self.serial_port_ = serial.Serial(port_name, baudrate=115200, timeout=3)
        print '{}: Connected to: {}'.format(__name__, port_name)
        return True


    # Method for seting wheel radius
    def set_wheel_diameter(self, wheel_diameter):
        self._wheel_radius = wheel_diameter / 2





    
    # Method for starting serial receive thread
    def run(self):
        self._rx_thread_.start()





    # Method for gracefully shutting down serial receive thread
    def shutdown(self, blocking=True):
        self._run_event.clear()
        if blocking:
            self._rx_thread_.join()




    # Method for transmitting commands through serial
    def send_command(self, v1, v2) :
        
        # Here we create and populate the message we want to send to the motor controller
        # Our message will contain the following 10 bytes:
        # [header, cmd1_high, cmd1_low, cmd2_high, cmd2_low, not_used, not_used, checksum_high, checksum_low, newline]
        # Values that cannot fit into one byte are split into two bytes (xx_high, xx_low) using bitwise logic
        cmd_1 = int(self.SPEED_MULTIPLIER * v1 / self._wheel_radius )
        cmd_2 = int(self.SPEED_MULTIPLIER * v2 / self._wheel_radius )
        tx_msg = [0] * self.MSG_SIZE

        tx_msg[0]  = ord('c')               # msg header
        tx_msg[1]  = (cmd_1 >> 8) & 0xff    # command, motor 1 high
        tx_msg[2]  = (cmd_1) & 0xff         # command, motor 1 low
        tx_msg[3]  = (cmd_2 >> 8) & 0xff    # command, motor 2 high
        tx_msg[4]  = (cmd_2) & 0xff         # command, motor 2 low
        tx_msg[-1] = ord('\n')              # finish message with newline

        
        crc = self._crc16(tx_msg[0:-3])      # calculate checksum
        
        
        tx_msg[-3] = (crc >> 8) & 0xff      # checksum high
        tx_msg[-2] = (crc) & 0xff           # checksum low

        
        # Send message
        self.serial_port_.write(tx_msg)





    # Returns latest measurement from distance sensor 1
    def get_dist_1(self):
        
        self._mutex.acquire()
        dist = self._dist_1
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_2(self):
        
        self._mutex.acquire()
        dist = self._dist_2
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_3(self):
        
        self._mutex.acquire()
        dist = self._dist_3
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_4(self):
        
        self._mutex.acquire()
        dist = self._dist_4
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_5(self):
        
        self._mutex.acquire()
        dist = 0 #self._dist_5
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_6(self):
        
        self._mutex.acquire()
        dist = 0 #self._dist_6
        self._mutex.release()
        
        return dist




    # Returns latest measurement from distance sensor 2
    def get_dist_7(self):
        
        self._mutex.acquire()
        dist = 0 #self._dist_7
        self._mutex.release()
        
        return dist




    def get_motor_speed(self):
        self._mutex.acquire()
        speed_feedback = [self._motor_1_speed, self._motor_2_speed]
        self._mutex.release()

        return speed_feedback




    def get_motor_current(self):
        self._mutex.acquire()
        current_feedback = [self._motor_1_current, self._motor_2_current]
        self._mutex.release()

        return speed_feedback
        

    def _convert_byte(self, byte):
        if byte > 127:
            return (256-byte) * (-1)
        else:
            return byte

    # Thread for receiving serial messages
    # This thread will run concurrently with other threads
    # This particular thread's only job is to listen to incomming messages
    # The readline() function is set to block until it gets a newline character
    # By hvaing it in a separate thread, it will not block other threads while waiting for incomming messages
    def _rx_thread(self) :

        while self._run_event.is_set() :
            rx_msg = self.serial_port_.read(size=self.MSG_SIZE)
            rx_msg = bytearray(rx_msg)
            
            if(len(rx_msg) == self.MSG_SIZE) :
                crc_calc = self._crc16(rx_msg[0:-3])
                crc_msg = (rx_msg[-3] & 0xff) << 8 | (rx_msg[-2] & 0xff)
                crc_ok = (crc_calc == crc_msg)

                if crc_ok and rx_msg[0] == ord('f'):
                    self._mutex.acquire()
                    self._dist_1 = (rx_msg[1] & 0xff)
                    self._dist_2 = (rx_msg[2] & 0xff)
                    self._dist_3 = (rx_msg[3] & 0xff)
                    self._dist_4 = (rx_msg[4] & 0xff)
                    self._motor_1_current = (rx_msg[-11] & 0xff) << 8 | (rx_msg[-10] & 0xff)
                    self._motor_2_current = (rx_msg[-9] & 0xff) << 8 | (rx_msg[-8] & 0xff)
                    self._motor_1_speed  = float(self._convert_byte(rx_msg[5] & 0xff) << 8 | (rx_msg[6] & 0xff)) / self.SPEED_MULTIPLIER * self._wheel_radius
                    self._motor_2_speed  = float(self._convert_byte(rx_msg[7] & 0xff) << 8 | (rx_msg[8] & 0xff)) / self.SPEED_MULTIPLIER * self._wheel_radius
                    self._mutex.release()
                elif crc_ok and rx_msg[0] == ord('e'):
                    print 'Error message received! Error is: {}'.format(rx_msg[1])
                else:
                    self.serial_port_.read(size=1)
                    


  


        print '{}: Serial receive thread has finished cleanly'.format(__name__)
        
        


    # Checksum algorithm
    # This algorithm takes in a list of bytes and returns a two byte checksum.
    # The checksum is calculated and included in the message on the transmitting side.
    # On the receiving side, the receiver calculates the checksum and compare the reulting value to the one included in the message.
    # If the values match, we trust that the data contained in the message is uncorrupted
    @staticmethod
    def _crc16(data_list) :
        crc = 0x0000;
        POLY = 0x8408

        if len(data_list) == 0 :
            return (~crc)

        for byte in data_list :
            byte = 0xff & byte
            for i in range(8) :
                if ((crc & 0x0001) ^ (byte & 0x0001)) :
                    crc = (crc >> 1) ^ POLY
                else :
                    crc = crc >> 1
                byte = byte >> 1
        
        return (crc)

          





def main(argv) :

    # Example program
    print 'Example program'
    if len(argv) == 1 : 
        port_name = '/dev/ttyACM0'
    else :
        port_name = argv[1]

    # Create motor serial object
    motor_serial = IMRTRobotSerial()

    # Open serial port, exit if serial port can't be opened
    connected = motor_serial.connect(port_name)
    if not connected:
        print 'Exiting program'
        sys.exit()
        
    # Spin receive thread
    motor_serial.run()


    # Now we will send some motor commands until the program is terminated by the user
    speed = 0

    try:
        while True :
            speed = (speed + 0.1) % 1
            motor_serial.send_command(speed, 1-speed)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print 'Terminated by user'

    finally:
        # Exit
        motor_serial.shutdown()
        print 'Exiting program'




if __name__ == '__main__' :
    main(sys.argv)

 

