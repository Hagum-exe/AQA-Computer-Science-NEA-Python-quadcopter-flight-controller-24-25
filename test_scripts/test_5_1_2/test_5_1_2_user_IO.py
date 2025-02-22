import time
import machine
from lib.imu import MPU6050
import toolkit
from toolkit import *


############################## CONSTANTS ################################
################## system presets for pico ###################
target_cycle_freq = 250   #target PID frequency in hz
cycle_time_seconds = 1/target_cycle_freq    #set target cycle time``
cycle_time_us = int(round(cycle_time_seconds*1000000, 0))   #convert cycle to in micro_seconds``


############ gyro constants ##########
calibration_size = 500  #calibration sample size

angle_roll_temp = 0   #temporary rotation angles along roll and pitch axis
angle_pitch_temp = 0

################ PID constants ###################
############## RATE contants ###############
rate_roll_kp = 1.0
rate_roll_ki = 0.0
rate_roll_kd = 0.0
rate_pitch_kp = rate_roll_kp
rate_pitch_ki = rate_roll_ki
rate_pitch_kd = rate_roll_kd

rate_yaw_kp = 2
rate_yaw_ki = 12
rate_yaw_kd = 0.0

rate_i_term_limit = 400   #I-term limit to prevent integral windup


############## ANGLE constants ##############
angle_roll_kp = 1
angle_roll_ki = 0
angle_roll_kd = 0
angle_pitch_kp = angle_roll_kp
angle_pitch_ki = angle_roll_ki
angle_pitch_kd = angle_roll_kd

angle_i_term_limit = 400 #I-term limit to prevent integral windup


############### horizontal velocity controller constants ################
velo_x_kp = 1.6 
velo_x_ki = 0
velo_x_kd = 0

velo_y_kp = velo_x_kp
velo_y_ki = velo_x_ki
velo_y_kd = velo_x_kd

velo_x_ki_limit = 400
velo_y_ki_limit = velo_x_ki_limit


################ vertical velocity controller constants ##################
vert_velo_kp = 1
vert_velo_ki = 0
vert_velo_kd = 0

vert_velo_ki_limit = 1000

############## PWM output constants ################
pwm_min = 1000
pwm_max = 2000


############## throttle settings ###################
throttle_idle = 1150  #minimum throttle for four mototrs to spin up but not lift up
max_throttle = 1800  #maximum throttle limit


#################PID controller class#########################
class PID_controller:
  def __init__(self, kp, ki, kd, ki_limit):   #instantiate PID_controller object
    self.__kp = kp    #set parameters as private attributes``
    self.__ki = ki
    self.__kd = kd
    self.__ki_limit = ki_limit
    
    self.__E = 0    #initialise error, integral and setpoint
    self.__I = 0
    self.__sp = 0
    
  def get_E(self):    #getter for error
    return self.__E
  
  def get_I(self):    #getter for integral
    return self.__I()

  def respond(self, sp, pv):    #pv as current position being passed in, sp is set point
    if (sp != self.__sp): #renew integral sum if new setpoint is given
      self.__I = 0    #initial integral set as when new setpoint given
      self.__sp = sp  #record setpoint as private attribute
    err = sp - pv   #calculate error 
    P = self.__kp * err   #calculate P, I and D terms
    I = self.__I + ((self.__ki * err)/2) * cycle_time_seconds
    I = max(min(I, self.__ki_limit), -self.__ki_limit) # constrain within I-term limits, prevents integral windup
    D = self.__kd * ((err - self.__E)/cycle_time_seconds)
    self.__E = err  #store error for the next function application
    self.__I = I    #store integral for the next function application
    return P + I + D    #return aggregated PID term


############################### Functions #################################
########### measure RX PWM pulse width ###########
def read_pulse_width(pin):    #measures pulse width of PWM signal from CRSF receiver
    while pin.value() == 0:   #wait when pulse is low
      pass
    start = time.time_ns()    #record start time when pulse is not low
    while pin.value() == 1:   #wait when pulse is high
        pass
    end = time.time_ns()      #record end time when pulse is low again
    duration = end - start    #calculate duration of high pulse
    return int(duration / 1000)   #return duration of high pulse is mili seconds
  
########### normalise values ##############
def normalise(value, original_min, original_max, new_min, new_max):
    new_value = new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))   #linearly scaled value to be normalised between two limits
    
    return min(max(new_value, new_min),  new_max)   #restrict values within range of new_min and new_max

#################################### RUN main function #################################
def run():   
  ################### LED and buzzer setup ####################
  led_pin = 22
  buzz_pin = 28
  num_leds = 4
  buzz_led_inst = buzz_led(buzz_pin, led_pin, num_leds)
  msg_inst = toolkit.msg(buzz_led_inst)
  
  
  msg_inst.event_msg('fc_start_up')  #message beginning flight controller function
  time.sleep(1)   #delay 
  
  machine.freq(250000000)   #set machine clock pulse to 250MHz. 
  print("Overclocked to 250,000,000")   #message over lock
  
  
  ################### Gyro calibration #######################
  mpu_online = False
  
  mpu_scl = 21   #MPU6050 scl and sda pins
  mpu_sda = 20
  mpu_i2c_num = 0   #I2C bus number used by MPU6050
  
  msg_inst.event_msg('gyro_calib')   #start gyro calibration
  
  mpu_measure_inst = mpu_measure(mpu_scl, mpu_sda, mpu_i2c_num)   #instantiate mpu_measure object
  mpu_measure_inst.calibrate(calibration_size)  #attempt to calibrate MPU6050

  msg_inst.event_msg('end_gyro_calib')   #message gyro calibration
  msg_inst.event_msg('mpu_online')   #message MPU6050 online
  mpu_online = True   #set gyroscope online as True
    
  #except:
  #  msg_inst.error_msg('mpu_online')

  ######################## RX set up ########################
  rx_online = False   #set initial online check as False
  
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
  
  try:
    pitch_input = read_pulse_width(pwm_channel_4)   #attempt to read from all channels
    roll_input = read_pulse_width(pwm_channel_5)
    throttle_input = read_pulse_width(pwm_channel_2), pwm_min, pwm_max, 0, 1
    yaw_input =read_pulse_width(pwm_channel_1)
    arm_input = read_pulse_width(pwm_channel_3)
    msg_inst.event_msg('rx_online')    #message rx online 
    rx_online = True    #set rx online as True
    
  except:
    msg_inst.error_msg('rx_online')    #if failed to read channel, show error message
  

  #################### PMW3901 and Arduino Nano check and calibration ####################
  nano_online = False   #set nano_online as false
  
  nano_scl = 27   #define Arduino Nano I2C pins
  nano_sda = 26   
  nano_num = 1    #define Arduino Nano I2C bus number
  
  
  horizontal_measure_inst = horizontal_measure(nano_scl, nano_sda, nano_num)    #attempt instantiate horizontal_measure object and calibrate horizontal velocity measurement
  horizontal_measure_inst.calibrate(calibration_size)
  nano_online = True    #set nano_online as true
  msg_inst.event_msg('nano_online')   #send event message nano_online
  
  #except:
  #  msg_inst.error_msg('nano_online')   #send error message nano_offline
    
    
  #################### BMP280 check and calibration ###################
  bmp_online = False    #set initial bmp_online check as False
  
  bmp_scl = 21    #define bmp280 pins
  bmp_sda = 20
  bmp_i2c_num = 0   #define bmp280 I2C bus number
  
  
  bmp_measure_inst = bmp_measure(bmp_scl, bmp_sda, bmp_i2c_num)   #instantiate bmp_measure object
  bmp_measure_inst.calibrate(calibration_size)  #calibrate bmp280 altitude
  bmp_online = True    #set bmp_online as true
  msg_inst.event_msg('bmp_online')  #message bmp_online message
    
  #except:
  #  pass
  print(mpu_online, rx_online, nano_online, bmp_online)
    
  
  if mpu_online and rx_online and nano_online and bmp_online:   #if all necessary devices are online
    ####################### setup motor pwm #####################
    pwm_output_freq = 50   #PWM output frequency as 50Hz
    pwm_motor1 = 3 #rear right, cw
    pwm_motor2 = 5 #front right, ccw
    pwm_motor3 = 7 #rear left, ccw
    pwm_motor4 = 9 #front left, cw
    
    m1 = machine.PWM(pwm_motor1)  #define motor pins
    m2 = machine.PWM(pwm_motor2)
    m3 = machine.PWM(pwm_motor3)
    m4 = machine.PWM(pwm_motor4)
    
    m1.freq(pwm_output_freq)    #define motor PWM frequency
    m2.freq(pwm_output_freq)
    m3.freq(pwm_output_freq)
    m4.freq(pwm_output_freq)
    
    m1.duty_ns(pwm_min*1000)    #set initial motor speed as 0
    m2.duty_ns(pwm_min*1000)
    m3.duty_ns(pwm_min*1000)
    m4.duty_ns(pwm_min*1000)
    
    ######### calculate throttle range ##############
    throttle_range = max_throttle - throttle_idle
    
    ############## instantiate PID control class ##################
    rate_PID_control_roll = PID_controller(rate_roll_kp, rate_roll_ki, rate_roll_kd,  rate_i_term_limit)    #roll rate PID controller
    rate_PID_control_pitch = PID_controller(rate_pitch_kp, rate_pitch_ki, rate_pitch_kd, rate_i_term_limit)   #pitch rate PID controller
    rate_PID_control_yaw = PID_controller(rate_yaw_kp, rate_yaw_ki, rate_yaw_kd, rate_i_term_limit)
    
    angle_PID_control_roll = PID_controller(angle_roll_kp, angle_roll_ki, angle_roll_kd, angle_i_term_limit)    #roll angle PID controller
    angle_PID_control_pitch = PID_controller(angle_pitch_kp, angle_pitch_ki, angle_pitch_kd, angle_i_term_limit)    #pitch angle PID controller
    
    velo_PID_controller_x = PID_controller(velo_x_kp, velo_x_ki, velo_x_kd, velo_x_ki_limit)    #x axis velocity PID controller
    velo_PID_controller_y = PID_controller(velo_y_kp, velo_y_ki, velo_y_kd, velo_y_ki_limit)    #y axis velocity PID controller
    
    vert_velo_PID_controller = PID_controller(vert_velo_kp, vert_velo_ki, vert_velo_kd, vert_velo_ki_limit)   #z axis vertical velocity PID controller
    
    armed = False   #set both armed and angle_mode as False, default air mode
    angle_mode = False
    
    ############## Instatiate measurement class(es) ####################
    measure_alt_kalman_inst = measure_alt_kalman(bmp_measure_inst=bmp_measure_inst, mpu_measure_inst=mpu_measure_inst)    #instntiate vertical measure_alt_kalman object
  
  
    ######################## BEGIN FC LOOP ################################
    #try:
    msg_inst.event_msg('begin_fc_loop')  #start flight control loop
    
    f = open('horiz_velo_IO_test.csv', 'w')
    
    while True:   #flight control loop executes forever until power is disconnected
        loop_begin_us:int = time.ticks_us()   # mark start time
        
        arm_input = read_pulse_width(pwm_channel_3)   #read arm input
        arm_input = normalise(arm_input, pwm_min, pwm_max, -1, 1)   #normalise arm input between -1 and 1
        
        #print(arm_input)
        ############################### DISARMED ##########################
        if arm_input>=1:
            t1 = pwm_min*1000   #turn off all motors
            t2 = pwm_min*1000
            t3 = pwm_min*1000
            t4 = pwm_min*1000
            
            if armed == True:   #if armed before
                print('disarmed')   #print disarmed
                armed = False   #set armed as False
            
        ############################### ARMED, AIR MODE ###############################  
        elif arm_input < 1 and arm_input >= -0.5:
            ########################## Measure angle rotation RATE #################
            angle_rate_roll, angle_rate_pitch, angle_rate_yaw = mpu_measure_inst.measure_rate()
            
            ############# Read desired rotation RATES ###########
            pitch_input = 0.15*(read_pulse_width(pwm_channel_4) - 1500)
            roll_input = 0.15*(read_pulse_width(pwm_channel_5) - 1500)
            throttle_input = read_pulse_width(pwm_channel_2)
            yaw_input = -0.15*(read_pulse_width(pwm_channel_1) - 1500)
            
            throttle_input = normalise(throttle_input, pwm_min, pwm_max, 0, 1)    #normalise throttle input
            throttle_input = throttle_idle + throttle_input*throttle_range    #increase throttle input to ensure above throttle_idle threshold
            
            ########## calculate roll, pitch, yaw rate setpoint ########
            rate_setpoint_roll = roll_input
            rate_setpoint_pitch= pitch_input
            rate_setpoint_yaw = yaw_input
            
            #print(f'{rate_setpoint_roll}, {rate_setpoint_yaw}, {rate_setpoint_pitch}, {throttle_input}')
            #print(f'{angle_rate_roll}, {angle_rate_pitch}, {angle_rate_yaw}')
            
            roll_resp = rate_PID_control_roll.respond(rate_setpoint_roll, angle_rate_roll)    #take response from rotation rate PID controllers
            pitch_resp = rate_PID_control_pitch.respond(rate_setpoint_pitch, angle_rate_pitch)
            yaw_resp = rate_PID_control_yaw.respond(rate_setpoint_yaw, angle_rate_yaw)
            
            #print(f'{roll_resp},{pitch_resp},{yaw_resp}')
            
            #print(f'{roll_resp}, {pitch_resp}, {yaw_resp}')
            
            ########## calculate throttle value ###########
            
            #print(f'{t1}, {t2}, {t3}, {t4}')
            
            t1 = min(max(throttle_idle, t1), max_throttle)    #limit throttle values between throttle idle and max_throttle
            t2 = min(max(throttle_idle, t2), max_throttle)
            t3 = min(max(throttle_idle, t3), max_throttle)
            t4 = min(max(throttle_idle, t4), max_throttle)
            
            t1 = int(round(t1))*1000    #calculate motor output in nano seconds
            t2 = int(round(t2))*1000
            t3 = int(round(t3))*1000
            t4 = int(round(t4))*1000
            
            #print(f'{t1}, {t2}, {t3}, {t4}')
            
            if armed == False:    #if was disarmed
                print('armed')    #print armed
                armed = True      #set armed to true
                
            if angle_mode == True:    #if was in angle mode
                print('air_mode')     #show in air mode
                angle_mode = False    #set angle mode to false
                
        ######################### ARMED, ANGLE mode with horizontal velocity control and vertical velocity control #########################
        elif arm_input <= -0.5:
            ############## Read desired horizontal speed ##############
            pitch_input = 0.1*(read_pulse_width(pwm_channel_4) - 1500)
            roll_input = 0.1*(read_pulse_width(pwm_channel_5) - 1500)
            throttle_input = read_pulse_width(pwm_channel_2)
            yaw_input = 0.1*(read_pulse_width(pwm_channel_1) - 1500)
            
            throttle_input = normalise(throttle_input, pwm_min, pwm_max, 0, 1)    #normalise throttle input
            throttle_input = throttle_idle + throttle_input*throttle_range    #increase throttle input to ensure above throttle_idle threshold
            
            ############### Read ACTUAL rotation angle ##############
            angle_roll, angle_pitch = mpu_measure_inst.measure_angle_degrees() 
            
            ############## Read ACTUAL rotation rate ############
            angle_rate_roll, angle_rate_pitch, angle_rate_yaw = mpu_measure_inst.measure_rate()
            
            ############## Read ACTUAL horizontal velocity #############
            velo_x, velo_y = horizontal_measure_inst.measure()
            
            ############## Read ACTUAL altitude ############
            alt, velo_z = measure_alt_kalman_inst.measure_alt()
            
            ################################## PID response ############################
            velo_sp_x = roll_input
            velo_sp_y = pitch_input
            rate_setpoint_yaw = yaw_input
            vert_velo_sp = throttle_input
            
            ########## compute rotation angle response from horizontal velocity PID controller ##########
            angle_sp_roll = velo_PID_controller_x.respond(velo_sp_x, velo_x)
            angle_sp_pitch = velo_PID_controller_y.respond(velo_sp_y, velo_y)
            
            #angle_sp_roll = roll_input
            #angle_sp_pitch = pitch_input
            
            ########### Compute rotation rate response from angle PID controller ############
            #angle_sp_roll = roll_input
            #angle_sp_pitch = pitch_input
            
            rate_setpoint_roll = angle_PID_control_roll.respond(angle_sp_roll, angle_roll)
            rate_setpoint_pitch = angle_PID_control_pitch.respond(angle_sp_pitch, angle_pitch)
            
            #print(f'Angle:{angle_roll}\t{rate_setpoint_roll}\t ----- Pitch:{angle_pitch}\t{rate_setpoint_pitch}\n')
            
            ########### Compute motor response from output of angle PID controller ##########
            roll_resp = rate_PID_control_roll.respond(rate_setpoint_roll, angle_rate_roll)
            pitch_resp = rate_PID_control_pitch.respond(rate_setpoint_pitch, angle_rate_pitch)
            #yaw_resp = rate_PID_control_yaw.respond(rate_setpoint_yaw, angle_rate_yaw)
            yaw_resp = 0
            ########### Compute throttle value from vertical velocity PID controller ##########
            throttle_resp = vert_velo_PID_controller.respond(vert_velo_sp, velo_z)
            
            
            
            ########### calculate throttle value ###############
            t1:float = throttle_resp + pitch_resp - roll_resp + yaw_resp
            t2:float = throttle_resp- pitch_resp - roll_resp - yaw_resp
            t3:float = throttle_resp + pitch_resp + roll_resp - yaw_resp
            t4:float = throttle_resp - pitch_resp + roll_resp + yaw_resp
            
            t1:float = throttle_input + pitch_resp - roll_resp + yaw_resp   #calculate thorttle value using motor mixing
            t2:float = throttle_input- pitch_resp - roll_resp - yaw_resp
            t3:float = throttle_input + pitch_resp + roll_resp - yaw_resp
            t4:float = throttle_input - pitch_resp + roll_resp + yaw_resp
            t1 = min(max(throttle_idle, t1), max_throttle)    #constrain throttle value within throttle_idel and max_throttle
            t2 = min(max(throttle_idle, t2), max_throttle)
            t3 = min(max(throttle_idle, t3), max_throttle)
            t4 = min(max(throttle_idle, t4), max_throttle)
            
            t1 = int(round(t1))*1000    #calcualte motor duty cycle in nano seconds
            t2 = int(round(t2))*1000
            t3 = int(round(t3))*1000
            t4 = int(round(t4))*1000
            
            msg = f'{velo_sp_x},{velo_sp_y},{angle_sp_roll},{angle_sp_pitch},{rate_setpoint_roll},{rate_setpoint_pitch},{roll_resp},{pitch_resp},{t1},{t2},{t3},{t4}'
            f.write(f'{msg}\n')
            print(msg)
            
            if armed == False:    #if was disarmed
                print('armed')    #print armed
                armed = True      #set armed to True
            
            if angle_mode == False:   #if was not in angle mode
                print('angle_mode')   #show in angle mode
                angle_mode = True     #set angle_mode to True
                   
        m1.duty_ns(t1)    #write duty cycle in nano seconds to motor
        m2.duty_ns(t2)
        m3.duty_ns(t3)
        m4.duty_ns(t4)
        
        ############################### maintain cycle frequency ###############################
        loop_end_us:int = time.ticks_us()   #record end of cycle 
        elapsed_us:int = loop_end_us - loop_begin_us    #calculate elapsed time
        if elapsed_us < cycle_time_us:    #if elapsed time is less than target cycle time
            time.sleep_us(cycle_time_us - elapsed_us)   #wait until target cycle time is met
    '''       
    except Exception as e:
      # before we do anything, turn the motors OFF
      m1.duty_ns(pwm_min*1000)
      m2.duty_ns(pwm_min*1000)
      m3.duty_ns(pwm_min*1000)
      m4.duty_ns(pwm_min*1000)

      # deinit
      m1.deinit()
      m2.deinit()
      m3.deinit()
      m4.deinit()
      #exc_type, exc_obj, exc_tb = sys.exc_info()
      #FATAL_ERROR(f'{str(e)}, line: {exc_tb.tb_lineno}')
      FATAL_ERROR(str(e))
    '''
    ############################ IF FAILED to begin FC LOOP ####################################
  else:
    print('Error starting loop')

################################# MAIN ###############################
run()












  













