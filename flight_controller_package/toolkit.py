########### import libraries ##############
from machine import Pin, I2C
import array, time
import rp2
import utime
from math import *
import math
from lib.imu import MPU6050
from bmp280 import BMP280I2C
from umatrix import matrix

cycle_time_seconds = 1/250  #set target cycle frequency as 250Hz. 

#########buzzer and leds################################
################ buzzer class ##################
class buzzer():
    def __init__(self, buzz_pin):   #instantiate buzzer object
        self.buzzpin = Pin(buzz_pin, Pin.OUT)
    
    def buzz(self, interval_ms):    #method to allow buzzer to buzz
        self.buzzpin.high() #set pin as high voltage
        time.sleep_ms(interval_ms)  #wait for set interval
        self.buzzpin.low()  #set pin as low voltage
        
###############################################

################### LEDs######################

############## led_strip class #################
class led_strip():  
    black = (0, 0, 0)   #define list of all colours stored as tuples of RGB values
    red = (255, 0, 0)
    yellow = (255, 150, 0)
    green = (0, 255, 0)
    cyan = (0, 255, 255)
    blue = (0, 0, 255)
    purple = (180, 0, 255)
    white = (255, 255, 255)
    colors_list = [black, red, yellow, green, cyan, blue, purple, white]
    
    def __init__(self, num_leds, pin_num, brightness):  #create led_strip object
        self.__num_leds = num_leds  #define number of leds, pin number used for led strip and brightness of led strip
        self.__pin_num = pin_num
        self.brightness = brightness
        
        @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
        def led_state_machine():   #defines PIO state machine
            T1 = 2  #set timing for high and low pulses
            T2 = 5
            T3 = 3
            wrap_target()   #bit manipulation to ensure correct representation of colour
            label("bitloop")
            out(x, 1)               .side(0)    [T3 - 1]
            jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
            jmp("bitloop")          .side(1)    [T2 - 1]
            label("do_zero")
            nop()                   .side(0)    [T2 - 1]
            wrap()
            
        self.__sm = rp2.StateMachine(0, led_state_machine, freq=8_000_000, sideset_base=Pin(self.__pin_num)) #creates state machine with led_sate_machine subroutine as argument
        self.__sm.active(1)
        # Display a pattern on the LEDs via an array of LED RGB values.
        self.__ar = array.array("I", [0 for _ in range(self.__num_leds)])
        
        
    def pixels_show(self):
        dimmer_ar = array.array("I", [0 for _ in range(self.__num_leds)])
        for i,c in enumerate(self.__ar):   #iterates through each pixel (individual leds) represented by the array
            r = int(((c >> 8) & 0xFF) * self.brightness) #extract RGB components from integer using mask, multiply by brightness to adjust brightness of leds
            g = int(((c >> 16) & 0xFF) * self.brightness)
            b = int((c & 0xFF) * self.brightness)
            dimmer_ar[i] = (g<<16) + (r<<8) + b #re-format given RGB values into GRB format required by WS2812
        self.__sm.put(dimmer_ar, 8)    #sends formatted and dimmed colours to leds
        time.sleep_ms(10)   #delay
        
    
    def pixels_set(self, i, color): #set pixel to a colour
        self.__ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]
        
    def pixels_fill(self, color_index): #fill list of pixels with colour
        color = led_strip.colors_list[color_index]
        for i in range(len(self.__ar)): #iterates through each pixel
            self.pixels_set(i, color)   #set pixel to given colour
            
    def flash(self, colour_index, interval_ms): #flash led with given time interval in ms
        self.pixels_fill(colour_index)  #fill pixel array with given colour
        self.pixels_show()  #show pixel
        time.sleep_ms(interval_ms)  #wait for given interval
        self.pixels_fill(0) #fill pixels to all black
        self.pixels_show()  #show pixel
        time.sleep_ms(interval_ms)  #wait for given interval
    
        
################################################################
#################### buzz_led class #######################
class buzz_led(buzzer, led_strip):  
    dot_ms = 50     #dot (showing 0) lasts 50ms
    dash_ms = 300   #dash (showing 1) lasts 300ms
    
    def __init__(self, buzz_pin, led_pin, num_leds, brightness=0.8):    #instantiates buzz_led object, brightness can be channged optionally
        buzzer.__init__(self, buzz_pin) #instantiate super classes
        led_strip.__init__(self, num_leds, led_pin, brightness)
        
        self.__error_dict = {'rx_error': '0000', 'mpu_error': '0001', 'nano_error': '0010',  'hc_error': '0011'}     #define dictionaries for error messages
        self.__event_dict = {'begin_fc_loop': '0100', 'fc_start_up': '0101', 'air_mode': '0110', 'angle_mode': '0111', 'armed': '0111', 'gyro_calib': '1001', 'end_gyro_calib': '1010', 'disarmed': '1011', 'rx_online':'1100', 'nano_online': '1101', 'bmp_online':'1110'}  #define dictionaries for event messages
        
    def get_error_event_dict(self): #getter for error and event messge dictionaries
        return self.__error_dict, self.__event_dict
    
    def set_error_event_dict(self, error_dict, event_dict): #setter for error and event messge dictionaries
        self.__error_dict = error_dict
        self.__event_dict = event_dict
        
    def flash_buzz(self, interval_ms, color_index):     #flash and bus with given interval
        self.buzz(interval_ms)
        self.flash(color_index, interval_ms)
        
    def error_msg(self, msg):   #shows error message with given message
        error_code = self.__error_dict[msg]
        for digit in error_code:
            if digit=='0':
                self.flash_buzz(buzz_led.dot_ms, 1)     #shows dot for a 0
            if digit=='1':
                self.flash_buzz(buzz_led.dash_ms, 1)    #shows dash for a 1
                
    def event_msg(self, msg):   #shows event message
        event_code = self.__event_dict[msg]
        for digit in event_code:
            if digit=='0':
                self.flash_buzz(buzz_led.dot_ms, 3)
            if digit=='1':
                self.flash_buzz(buzz_led.dash_ms, 3)

###############buzzer and leds end##########################


#################### MPU6050 rotation rate and angle measurement ###########################
class mpu_measure():
    def __init__(self, mpu_i2c_scl, mpu_i2c_sda, i2c_num, i2c_addr=0x68):
      self.__i2c_addr = i2c_addr   #specify I2C address, optionally changeable
      
      i2c = I2C(i2c_num, sda=Pin(mpu_i2c_sda), scl=Pin(mpu_i2c_scl), freq=200000)   #initialise I2C instance by instantiating Pin
      self.imu = MPU6050(i2c, self.__i2c_addr) #initialise MPU6050 instance
      
      self.__bias_rate_roll = 0   #set initial bias as 0 for both rate and angle calibration
      self.__bias_rate_pitch = 0
      self.__bias_rate_yaw = 0
      
      self.__bias_angle_roll_rads = 0
      self.__bias_angle_pitch_rads = 0
      self.__bias_angle_roll_degrees = 0
      self.__bias_angle_pitch_degrees = 0
      
      self.angle_roll_temp = 0  #set tempeorary angle and roll measurements as 0
      self.angle_pitch_temp = 0
        
    def get_i2c_addr(self):   #getter for I2C address
        return self.__i2c_addr
        self.__i2c_addr = i2c_addr
        
    def set_i2C_addr(self, i2c_addr):   #setter for I2C address
        self.__i2c_addr = i2c_addr
    
    def get_bias_rate(self):    #get bias in roll, pitch and yaw rate
        return self.__bias_rate_roll, self.__bias_rate_pitch, self.__bias_rate_yaw
    
    def get_bias_angle_rads(self):  #get bias of rotation angle in radians
        return self.__bias_angle_roll_rads, self.__bias_angle_pitch_rads

    def get_bias_angle_degrees(self):   #get bias of rotation angel in degrees
        return self.__bias_angle_roll_degrees, self.__bias_angle_pitch_degrees
       
    def calibrate(self, calibration_size):  #calibrate both rotation rate and angle with size specified by calibration_size
      ########  calibrate rotation rate ##########
      self.__bias_rate_roll = 0   #initiate all rotation rate biases as 0
      self.__bias_rate_pitch = 0
      self.__bias_rate_yaw = 0
      for i in range(0, calibration_size):  #take a sample of rotation rates of size specified in calibrartion_size
          gx = round(self.imu.gyro.x) 
          gy = round(self.imu.gyro.y) 
          gz = -round(self.imu.gyro.z) 

          self.__bias_rate_roll += gx #sum up all rotation rates
          self.__bias_rate_pitch += gy
          self.__bias_rate_yaw += gz
      
      self.__bias_rate_roll = self.__bias_rate_roll / calibration_size  #calculate mean of rotation rates and assign rotation rates as temperary rotation rates
      self.__bias_rate_pitch = self.__bias_rate_roll / calibration_size
      self.__bias_rate_yaw = self.__bias_rate_roll / calibration_size
      ################################################################
      
      ########  calibrate rotation angle ##########
      bias_angle_roll_rads = 0  #set initial rotation angel biases to 0
      bias_angle_pitch_rads = 0
        
      for i in range(0, calibration_size):  #take smaples of rotation angel specified in calibrartion_size
        roll, pitch = self.measure_angle_rads()
        bias_angle_roll_rads += roll    #sum up all rotation angles
        bias_angle_pitch_rads += pitch
            
      self.__bias_angle_roll_rads = bias_angle_roll_rads / calibration_size     #calcualte mean rotation angel bias
      self.__bias_angle_pitch_rads = bias_angle_pitch_rads / calibration_size
        
      self.__bias_angle_roll_degrees = self.__bias_angle_roll_rads*(180/math.pi)  #convert rotation angle bias in radians to degrees
      self.__bias_angle_pitch_degrees = self.__bias_angle_pitch_rads*(180/math.pi)
      ################################################################
      
      
    def measure_rate(self): #measure rotation rate corrected using rotation rate bias
      gx = round(self.imu.gyro.x) - self.__bias_rate_roll
      gy = round(self.imu.gyro.y) - self.__bias_rate_pitch
      gz = -round(self.imu.gyro.z) - self.__bias_rate_yaw
      
      return gx, gy, gz
    
    def measure_acc(self):  #measure acceleration along x, y and z axis
      ax = round(self.imu.accel.x, 2)  #read acceleration along axis
      ay = round(self.imu.accel.y, 2)
      az = round(self.imu.accel.z, 2)
      return ax, ay, az
      
      
    def measure_angle_degrees(self):    #measrue rotation angel in degrees
        ax = round(self.imu.accel.x, 2)  #read acceleration along axis
        ay = round(self.imu.accel.y, 2)
        az = round(self.imu.accel.z, 2)
       
        try:
            angle_roll =  math.atan(ay/(math.sqrt(ax**2+az**2)))    #calculate rotation angle along roll and pitch axis
            angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))
            
            self.angle_roll_temp = angle_roll   #set temporary rotation angles as measured rotation angles
            self.angle_pitch_temp = angle_pitch
             
        except:
            pass
            
        return self.angle_roll_temp*(180/math.pi) - self.__bias_angle_roll_degrees, self.   angle_pitch_temp*(180/math.pi) - self.__bias_angle_pitch_degrees    #return corrected rotation angles in degrees
    
    
    def measure_angle_rads(self):
        ax = round(self.imu.accel.x, 2)  #read acceleration along axis
        ay = round(self.imu.accel.y, 2)
        az = round(self.imu.accel.z, 2)
        
        try:
            angle_roll =  math.atan(ay/(math.sqrt(ax**2+az**2)))    #calculate rotation angle along roll and pitch axis
            angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))
            
            self.angle_roll_temp = angle_roll   #set temporary rotation angles as measured rotation angles
            self.angle_pitch_temp = angle_pitch
             
        except:
            pass
            
        return self.angle_roll_temp - self.__bias_angle_roll_rads, self.angle_pitch_temp  - self.__bias_angle_pitch_rads    #return corrected rotation angles
    
    
    def inert_acc_measure(self):    #measure inertial acceleration along z-axis
        ax, ay, az = self.measure_acc()     #measure acceleration along x, y and z axis
        angle_roll, angle_pitch = self.measure_angle_rads() #measure rotaion angle along roll and pitch axis in radians
        acc_z_inertial = -sin(angle_pitch)*ax + cos(angle_pitch)*sin(angle_roll)*ay+cos(angle_pitch)*cos(angle_roll)*az     #calcualte inertial acceleration
        acc_z_inertial = (acc_z_inertial-1)*9.81*100    
        return acc_z_inertial   #return inertial acceleration

################## measure horizontal velocity with PMW3901 and Arduino Nano #######################
class horizontal_measure():
    
    def __init__(self, nano_scl, nano_sda, i2c_num):    #create horizontal_measure object
        self.velo_x = 0 #set initial velocity as 0
        self.velo_y = 0
        
        self.__velo_bias_x = 0  #set initial velocity bias as 0
        self.__velo_bias_y = 0  
        
        self.__msg_size = 8 #message from ARduino is set as size of 8 characters
        
        self.i2c = I2C(i2c_num, scl=Pin(nano_scl), sda=Pin(nano_sda), freq=100000)    #initiate i2c communication with Arduino Nano
        self.i2c_addr = self.i2c.scan()[0]  #use the 1st device found on the I2C bus as communication target
    
    def get_velo_bias(self):    #getter for velocity bias
        return self.__velo_bias_x, self.__velo_bias_y
    
    def get_msg_size(self): #getter for message size
        return self.__msg_size 
    
    def set_msg_size(self, msg_size):   #setter for message size
        self.__msg_size = msg_size
    
    def calibrate(self, calibration_size):  #calibration method
        velo_bias_x_temp = 0    #set temporary velocity bias as 0
        velo_bias_y_temp = 0
        for i in range(calibration_size):   #take measurements specified by calibration_size
            velo_x, velo_y = self.measure()
            velo_bias_x_temp += velo_x  #sum up all measurements
            velo_bias_y_temp += velo_y
            
        self.__velo_bias_x = velo_bias_x_temp / calibration_size    #calculate mean measurement
        self.__velo_bias_y = velo_bias_y_temp / calibration_size
    
    def measure(self):  #method to measure horizontal velocity
        msg = self.i2c.readfrom(self.i2c_addr, self.__msg_size)   #read message from Nano on demand
        msg = str(msg)[2::] #format message to extract desired velocity
        msg = msg.split(";")
        
        try:
            self.velo_x = -float(msg[0])    #attempt to extract velocity
            self.velo_y = float(msg[1])
            
        except:
            pass
        
        return self.velo_x-self.__velo_bias_x, self.velo_y-self.__velo_bias_y   #return corrected horizontal velocity
    
################################################################################


########################## vertical kalman filter class ######################
class vertical_kalman():
    def __init__(self): #instnatiate vertical_kalman object
        self.__S = matrix([0], [0])  #state space matrix
        self.__F = matrix([1, cycle_time_seconds], [0, 1])  #state transition matrix
        self.__G = matrix([0.5*(cycle_time_seconds**2)], [cycle_time_seconds])    #control matrix
        self.__H = matrix([1, 0])   #observation matrix
        self.__I = matrix([1,0], [0,1]) #identity matrix
        self.__Q = self.__G*self.__G.transpose*(30**2)  #process uncertinaty 16.96
        self.__R = matrix([10**2])  #measurement uncertainty
        self.__P = matrix([0,0], [0,0]) #predicted uncretainty
        
        self.__alt = matrix([0])    #extract altitude
        self.__L = 0    #intermediate meatrices for operation
        self.__K = 0
           
    def kalman_filter(self, acc, alt):
        self.__S = self.__F*self.__S + self.__G*acc #equation 4.3.2.1
        self.__P = self.__F*self.__P*self.__F.transpose+self.__Q  #equation 4.3.2.2
        
        #print(self.F*self.P*self.F.transpose)
        
        self.__L = self.__H*self.__P*self.__H.transpose+self.__R  #equation 4.3.2.3
        self.__K = self.__P*self.__H.transpose*self.__L.inverse #equation 4.3.2.4
        
        self.__alt = matrix([alt])
        
        ############ Update statespace matrix and predicted state #################
        self.__S = self.__S + self.__K*(self.__alt-self.__H*self.__S)   #equation 4.3.2.5
        alt_kalman = self.__S[0][0]
        velo_kalman = self.__S[1][0]
        
        self.__P = (self.__I-self.__K*self.__H)*self.__P  #equation 4.3.2.6
        
        return alt_kalman, velo_kalman  #return filtered altitude and velocity

   
############################### bmp_measure class #################################
class bmp_measure():
    def __init__(self, i2c_scl, i2c_sda, i2c_num, i2c_addr = 0x76):
        self.__i2c_addr = i2c_addr    #specify i2c address of BMP280, default 0x76
        i2c = I2C(i2c_num, sda=Pin(i2c_sda), scl=Pin(i2c_scl), freq=400000) #initialise I2C instance for BMP280 by passing Pin objects as argument
        
        self.bmp280_i2c = BMP280I2C(self.__i2c_addr, i2c)  #initialise BMP280I2C instance
        self.__alt_bias = 0   #set initial altitude bias to 0
        
    def set_i2c_addr(self, i2c_addr):   #setter for I2C address
        self.__i2c_addr = i2c_addr
        
    def get_i2c_addr(self):     #getter for I2C address
        return self.__i2c_addr
    
    def get_alt_bias(self):     #getter for altiude bias
        return self.__alt_bias

    def measure(self):      #measure bmp280 altitude
        readout = self.bmp280_i2c.measurements

        pressure = readout['p']
        
        alt = 4433000*(1-(pressure/1013.25)**(1/5.255))
        
        return alt-self.__alt_bias
    
    def calibrate(self, calibration_size):      #calibrate bmp280 altitude
        alt_bias = 0    #set initial temporary altitude bias to 0
        for i in range(0, calibration_size):    #collect number of smaples specified by calibration_size
            alt = self.measure()
            alt_bias += alt     #sum up all altitude measurements
            
        alt_bias = alt_bias / calibration_size  #calculate mean altitude
        self.__alt_bias = alt_bias  #set altitude bias
########################################################################

      
############################# measure_alt_kalman #########################
class measure_alt_kalman():
    def __init__(self, bmp_measure_inst, mpu_measure_inst):     #instantiate measure_alt_kalman object
        self.bmp_measure = bmp_measure_inst     #assign aggregated bmp_measure object
        self.mpu_measure = mpu_measure_inst     #assign aggregated mpu_measure object
        self.__vertical_kalman = vertical_kalman()      #instantiate vertical_kalman() object
        
    def measure_alt(self):      #method to measure altitude
        alt = self.bmp_measure.measure()    #raw altitude measurement taken from bmp_measure
        acc_z_inert = self.mpu_measure.inert_acc_measure()      #inertial acceleration along z-axis measured using mpu_measure
        alt_kalman, velo_kalman = self.__vertical_kalman.kalman_filter(acc_z_inert, alt)    #filter vertical velocity and altitude
        
        return alt_kalman, velo_kalman  #return filtered altitude and velocity
 
###############logging, error, and event messages#################
class msg():
    def __init__(self, buzz_led_inst):
        self.__buzz_led_inst = buzz_led_inst
        
    def error_msg(self, msg):
        error_msg = "Fatal error @ " + str(time.ticks_ms()) + " ms: " + msg   #compose error message with time stamp
        print(error_msg)    #print error message, viewable if connected to terminal
        self.log(error_msg)  #log error message using logger in toolki.py
        try: self.__buzz_led_inst.error_msg(msg)   #attempt to send error message
        except: pass
        
    def event_msg(self, msg):
        event_msg = "Event @ "+ str(time.ticks_ms()) + " ms: " + msg    #compose event message with time stamp
        print(event_msg)    #print event message, viewable if connected to terminal
        self.log(event_msg)    #log event message using logger in toolki.py
        try: self.__buzz_led_inst.event_msg(msg)  #attempt to send event message
        except: pass

    def log(self, msg):
        f = open('logs.txt', 'a')   #open logs in append mode
        f.write(msg + '\n\n')   #append message to file
        f.close()   #close file



