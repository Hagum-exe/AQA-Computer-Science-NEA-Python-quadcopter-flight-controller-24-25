import time
from machine import I2C, Pin
import machine
import math
from math import sin, cos, tan
from lib.imu import MPU6050
import time


pwm_pin_1 = 2   #setup RX pints
pwm_pin_2 = 4
pwm_pin_3 = 6
pwm_pin_4 = 8
pwm_pin_5 = 10

pwm_channel_1 = machine.Pin(pwm_pin_1, Pin.IN)   #set PWM pins in read mode
pwm_channel_2 = machine.Pin(pwm_pin_2, Pin.IN)
pwm_channel_3 = machine.Pin(pwm_pin_3, Pin.IN)
pwm_channel_4 = machine.Pin(pwm_pin_4, Pin.IN)
pwm_channel_5 = machine.Pin(pwm_pin_5, Pin.IN)

def read_pulse_width(pin):    #measures pulse width of PWM signal from CRSF receiver
    while pin.value() == 0:   #wait when pulse is low
      pass
    start = time.time_ns()    #record start time when pulse is not low
    while pin.value() == 1:   #wait when pulse is high
        pass
    end = time.time_ns()      #record end time when pulse is low again
    duration = end - start    #calculate duration of high pulse
    return int(duration / 1000)   #return duration of high pulse is mili seconds

def normalise(value, original_min, original_max, new_min, new_max):
    new_value = new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))   #linearly scaled value to be normalised between two limits
    
    return min(max(new_value, new_min),  new_max)   #restrict values within range of new_min and new_max

pwm_min = 1000
pwm_max = 2000

f = open('rx_channels.csv', 'w')

print('Start sampling')
time.sleep(1)

for i in range(0, 1000):
    pitch_input = 0.15*(read_pulse_width(pwm_channel_4) - 1500)
    roll_input = 0.15*(read_pulse_width(pwm_channel_5) - 1500)
    yaw_input = -0.15*(read_pulse_width(pwm_channel_1) - 1500)
    
    throttle_input = read_pulse_width(pwm_channel_2)
    throttle_input = normalise(throttle_input, pwm_min, pwm_max, 0, 1)
    
    arm_input = read_pulse_width(pwm_channel_3)
    arm_input = normalise(arm_input, pwm_min, pwm_max, -1, 1)
    
    msg = f'{pitch_input},{roll_input},{yaw_input},{throttle_input},{arm_input}'
    f.write(msg)
    print(msg)
    
    
    
f.close()
    



