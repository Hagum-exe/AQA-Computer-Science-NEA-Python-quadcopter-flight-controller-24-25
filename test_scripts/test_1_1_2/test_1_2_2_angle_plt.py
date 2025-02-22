import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('angle_motor_pid_IO_test.csv')
print(df.head)
index = list(df.index.values)
pitch_input_list = df['pitch_input'].values.tolist()
roll_input_list = df['roll_input'].values.tolist()

pitch_rate_sp_list = df['rate_sp_pitch'].values.tolist()
roll_rate_sp_list = df['rate_sp_roll'].values.tolist()

pitch_resp_list = df['pitch_resp'].values.tolist()
roll_resp_list = df['roll_resp'].values.tolist()

t1_list = df['t1'].values.tolist()
t2_list = df['t2'].values.tolist()
t3_list = df['t3'].values.tolist()
t4_list = df['t4'].values.tolist()

'''
figure, axis = plt.subplots(2, 1)

axis[0].set_title('roll_input')
axis[1].set_title('roll_resp')

axis[0].plot(index, roll_input_list, color = 'orange')
axis[1].plot(index, roll_resp_list, color = 'blue')
'''


figure, axis = plt.subplots(7, 1)

axis[0].set_title('roll_input')
axis[1].set_title('roll_rate_sp')
axis[2].set_title('roll_resp')
axis[3].set_title('t1')
axis[4].set_title('t2')
axis[5].set_title('t3')
axis[6].set_title('t4')

axis[0].plot(index, roll_input_list, color = 'orange')
axis[1].plot(index, roll_rate_sp_list, color = 'blue')
axis[2].plot(index, roll_resp_list, color = 'grey')
axis[3].plot(index, t1_list, color = 'green')
axis[4].plot(index, t2_list, color = 'green')
axis[5].plot(index, t3_list, color = 'green')
axis[6].plot(index, t4_list, color = 'green')


plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




