import time
from machine import I2C, Pin
import math
from math import sin, cos, tan
from lib.imu import MPU6050
import time

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
        return -acc_z_inertial   #return inertial acceleration


scl = 21    #define MPU6050 scl pin
sda = 20    #define MPU6050 sda pin
i2c_num = 0

mpu_measure_inst = mpu_measure(scl, sda, i2c_num)
mpu_measure_inst.calibrate(500)
f = open('inert_acc_test.csv', 'w')

print('Start sampling')
time.sleep(1)

for i in range(0, 1000):
    inert_acc = mpu_measure_inst.inert_acc_measure()
    print(inert_acc)
    f.write(f'{inert_acc}\n')
    
f.close()
    



