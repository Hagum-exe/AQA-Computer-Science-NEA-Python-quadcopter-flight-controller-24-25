import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('horiz_velo_stick_IO_test.csv')
print(df.head)
index = list(df.index.values)

velo_sp_x_list = df['velo_sp_x'].values.tolist()
velo_sp_y_list = df['velo_sp_y'].values.tolist()

angle_sp_roll_list = df['angle_sp_roll'].values.tolist()
angle_sp_pitch_list = df['angle_sp_pitch'].values.tolist()

rate_sp_roll_list = df['rate_sp_roll'].values.tolist()
rate_sp_pitch_list = df['rate_sp_pitch'].values.tolist()

roll_resp_list = df['roll_resp'].values.tolist()
pitch_resp_list = df['pitch_resp'].values.tolist()

t1_list = df['t1'].values.tolist()
t2_list = df['t2'].values.tolist()
t3_list = df['t3'].values.tolist()
t4_list = df['t4'].values.tolist()

figure, axis = plt.subplots(8, 1)

'''
axis[0].set_title('velo_sp_x')
axis[1].set_title('angle_sp_roll')
axis[2].set_title('rate_sp_roll')
axis[3].set_title('roll_resp')
axis[4].set_title('t1')
axis[5].set_title('t2')
axis[6].set_title('t3')
axis[7].set_title('t4')

axis[0].plot(index, velo_sp_x_list, color = 'orange')
axis[1].plot(index, angle_sp_roll_list, color = 'blue')
axis[2].plot(index, rate_sp_roll_list, color = 'grey')
axis[3].plot(index, roll_resp_list, color = 'red')
axis[4].plot(index, t1_list, color = 'green')
axis[5].plot(index, t2_list, color = 'green')
axis[6].plot(index, t3_list, color = 'green')
axis[7].plot(index, t4_list, color = 'green')
'''


axis[0].set_title('velo_sp_y')
axis[1].set_title('angle_sp_pitch')
axis[2].set_title('rate_sp_pitch')
axis[3].set_title('pitch_resp')
axis[4].set_title('t1')
axis[5].set_title('t2')
axis[6].set_title('t3')
axis[7].set_title('t4')

axis[0].plot(index, velo_sp_y_list, color = 'orange')
axis[1].plot(index, angle_sp_pitch_list, color = 'blue')
axis[2].plot(index, rate_sp_pitch_list, color = 'grey')
axis[3].plot(index, pitch_resp_list, color = 'red')
axis[4].plot(index, t1_list, color = 'green')
axis[5].plot(index, t2_list, color = 'green')
axis[6].plot(index, t3_list, color = 'green')
axis[7].plot(index, t4_list, color = 'green')


list_x = [0, 200]
list_y = [0,0]

for i in range(0, 4):
    axis[i].plot(list_x, list_y, linestyle = 'dashed', color = 'black')

figure.tight_layout(pad=5)

plt.legend(fontsize = '10')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




