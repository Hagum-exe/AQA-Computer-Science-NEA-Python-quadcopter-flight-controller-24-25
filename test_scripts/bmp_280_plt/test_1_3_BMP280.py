from lib.bmp280 import BMP280I2C
import time
from machine import I2C, Pin

class bmp_measure():
    def __init__(self, i2c_scl, i2c_sda, i2c_num):
        i2c_addr = 0x76    #specify i2c address of BMP280
        i2c = I2C(i2c_num, sda=Pin(i2c_sda), scl=Pin(i2c_scl), freq=400000) #initialise I2C instance for BMP280
        
        self.bmp280_i2c = BMP280I2C(i2c_addr, i2c)  #initialise BMP280I2C instance
        self.alt_bias = 0   #set initial altitude bias to 0
        
        

    def measure_alt(self):
        readout = self.bmp280_i2c.measurements  #extract pressure from bmp280 pressure read out

        pressure = readout['p']
        
        alt = 4433000*(1-(pressure/1013.25)**(1/5.255)) #calculate altitude with measured air pressure
        
        return alt-self.alt_bias    #return calibrated altitude
    
    def calibrate(self, calibration_size):
        alt_bias = 0    #set initial altitude bias as 0
        for i in range(0, calibration_size):    #collect samples as specified by argument
            alt = self.measure_alt()
            alt_bias += alt #increment altitude bias by collected altitude
            
        alt_bias = alt_bias / calibration_size  #calculate mean static altitude
        self.alt_bias = alt_bias    #set object attribute as calculated mean bias

scl = 21    #define SCL and SDA pins
sda = 20
bmp_measure_inst = bmp_measure(scl, sda, 0) #instantiate instance of bmp_measure
bmp_measure_inst.calibrate(300) #calibrate with 300 samples 
f = open('alt_test.csv', 'w')   #open file alt_test.csv
for i in range(0, 1000):    #take 1000 measurements post calibration
    alt = bmp_measure_inst.measure_alt()    #write each sample to file
    f.write(f'{alt}\n')
    time.sleep(0.05)    #delay 
    
f.close()   #close file
