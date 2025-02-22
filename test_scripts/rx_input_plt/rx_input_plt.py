import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('rx_channels.csv')
print(df.head)
index = list(df.index.values)
pitch_list = df['pitch'].values.tolist()
roll_list = df['roll'].values.tolist()
yaw_list = df['yaw'].values.tolist()
throttle_list = df['throttle'].values.tolist()
arm_list = df['arm'].values.tolist()


figure, axis = plt.subplots(5, 1)

axis[0].set_title('pitch')
axis[1].set_title('roll')
axis[2].set_title('yaw')
axis[3].set_title('throttle')
axis[4].set_title('arm')


axis[0].plot(index, pitch_list, color = 'blue')
axis[1].plot(index, roll_list, color = 'blue')
axis[2].plot(index, yaw_list, color = 'blue')
axis[3].plot(index, throttle_list, color = 'orange')
axis[4].plot(index, arm_list, color = 'red')


plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




