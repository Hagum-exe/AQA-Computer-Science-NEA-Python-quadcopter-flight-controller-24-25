from lib.bmp280 import BMP280I2C
from lib.imu import MPU6050
import time
from machine import I2C, Pin
import math
class mpu_measure():
    def __init__(self, mpu_i2c_scl, mpu_i2c_sda, i2c_num):
      i2c_addr = 0x68   #specify I2C address
      
      i2c = I2C(i2c_num, sda=machine.Pin(mpu_i2c_sda), scl=machine.Pin(mpu_i2c_scl), freq=200000)   #initialise I2C instance
      self.imu = MPU6050(i2c, i2c_addr) #initialise MPU6050 instance
      
      self.bias_rate_roll = 0   #set initial bias as 0
      self.bias_rate_pitch = 0
      self.bias_rate_yaw = 0
      
      self.bias_angle_roll_rads = 0
      self.bias_angle_pitch_rads = 0
      self.bias_angle_roll_degrees = 0
      self.bias_angle_pitch_degrees = 0
      
      self.angle_roll_temp = 0
      self.angle_pitch_temp = 0
        
        
    def calibrate_rate(self, calibration_size):
      self.bias_rate_roll = 0   #initiate all rotation rate biases as 0
      self.bias_rate_pitch = 0
      self.bias_rate_yaw = 0
      for i in range(0, calibration_size):  #take a sample of rotation rates of size specified in calibrartion_size
          gx = round(self.imu.gyro.x) 
          gy = round(self.imu.gyro.y) 
          gz = -round(self.imu.gyro.z) 

          self.bias_rate_roll += gx #sum up all rotation rates
          self.bias_rate_pitch += gy
          self.bias_rate_yaw += gz
      
      self.bias_rate_roll = self.bias_rate_roll / calibration_size  #calculate mean of rotation rates
      self.bias_rate_pitch = self.bias_rate_roll / calibration_size
      self.bias_rate_yaw = self.bias_rate_roll / calibration_size
      
    def calibrate_angle(self, calibration_size):
        bias_angle_roll_rads = 0
        bias_angle_pitch_rads = 0
        
        for i in range(0, calibration_size):
            roll, pitch = self.measure_angle_rads()
            bias_angle_roll_rads += roll
            bias_angle_pitch_rads += pitch
            
        self.bias_angle_roll_rads = bias_angle_roll_rads / calibration_size
        self.bias_angle_pitch_rads = bias_angle_pitch_rads / calibration_size
        
        self.bias_angle_roll_degrees = self.bias_angle_roll_rads*(180/math.pi)
        self.bias_angle_pitch_degrees = self.bias_angle_pitch_rads*(180/math.pi)
    
    
    def measure_rate(self):
      gx = round(self.imu.gyro.x) - self.bias_rate_roll
      gy = round(self.imu.gyro.y) - self.bias_rate_pitch
      gz = -round(self.imu.gyro.z) - self.bias_rate_yaw
      
      return gx, gy, gz
    
    def measure_acc(self):
      ax = round(self.imu.accel.x, 2)  #read acceleration along axis
      ay = round(self.imu.accel.y, 2)
      az = round(self.imu.accel.z, 2)
      return ax, ay, az
      
      
    def measure_angle_degrees(self):
        ax = round(self.imu.accel.x, 2)  #read acceleration along axis
        ay = round(self.imu.accel.y, 2)
        az = round(self.imu.accel.z, 2)
        gx = round(self.imu.gyro.x) 
        gy = round(self.imu.gyro.y) 
        gz = -round(self.imu.gyro.z) 

        try:
            angle_roll =  math.atan(ay/(math.sqrt(ax**2+az**2)))
            angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))
            
            self.angle_roll_temp = angle_roll
            self.angle_pitch_temp = angle_pitch
             
        except:
            pass
            
        return self.angle_roll_temp*(180/math.pi) - self.bias_angle_roll_degrees, self.angle_pitch_temp*(180/math.pi) - self.bias_angle_pitch_degrees

    def measure_angle_rads(self):
            ax = round(self.imu.accel.x, 2)  #read acceleration along axis
            ay = round(self.imu.accel.y, 2)
            az = round(self.imu.accel.z, 2)
            gx = round(self.imu.gyro.x) 
            gy = round(self.imu.gyro.y) 
            gz = -round(self.imu.gyro.z) 

            
            try:
                angle_roll =  math.atan(ay/(math.sqrt(ax**2+az**2)))
                angle_pitch = math.atan(-ax/(math.sqrt(ay**2+az**2)))
                
                self.angle_roll_temp = angle_roll
                self.angle_pitch_temp = angle_pitch
                 
            except:
                pass
                
            return self.angle_roll_temp - self.bias_angle_roll_rads, self.angle_pitch_temp  - self.bias_angle_pitch_rads

mpu_scl = 21    #define MPU6050 scl pin
mpu_sda = 20    #define MPU6050 sda pin
mpu_measure_inst = mpu_measure(mpu_scl, mpu_sda, 0) #create mpu_measure instance
mpu_measure_inst.calibrate_rate(500)    #calibrate imu
mpu_measure_inst.calibrate_angle(500)   

f = open('roll_measurement_test.csv', 'w')  #open record file
for i in range(0, 1000):    
    angle = mpu_measure_inst.measure_angle_rads()[0]    #extract rotation angle along x-axis
    rate = mpu_measure_inst.measure_rate()[0]   #extract rotation rate along x-axis
    print(rate)
    print(angle)
    f.write(f'{angle}, {rate}\n')   #write rotation rate and angle to file
    time.sleep(0.05)    #delay
    
f.close()   #close file
