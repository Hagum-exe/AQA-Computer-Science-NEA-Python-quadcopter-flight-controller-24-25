import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('rate_motor_IO_test.csv')
print(df.head)
index = list(df.index.values)
pitch_input_list = df['pitch_input'].values.tolist()
roll_input_list = df['roll_input'].values.tolist()
yaw_input_list = df['yaw_input'].values.tolist()

pitch_resp_list = df['pitch_resp'].values.tolist()
roll_resp_list = df['roll_resp'].values.tolist()
yaw_resp_list = df['yaw_resp'].values.tolist()

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


figure, axis = plt.subplots(6, 1)

axis[0].set_title('yaw_input')
axis[1].set_title('yaw_resp')
axis[2].set_title('t1')
axis[3].set_title('t2')
axis[4].set_title('t3')
axis[5].set_title('t4')

axis[0].plot(index, yaw_input_list, color = 'orange')
axis[1].plot(index, yaw_resp_list, color = 'blue')
axis[2].plot(index, t1_list, color = 'green')
axis[3].plot(index, t2_list, color = 'green')
axis[4].plot(index, t3_list, color = 'green')
axis[5].plot(index, t4_list, color = 'green')


plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




