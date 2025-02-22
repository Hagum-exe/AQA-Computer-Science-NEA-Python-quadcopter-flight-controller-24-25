import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('vert_velo_IO_test.csv')
print(df.head)
index = list(df.index.values)

vert_velo_list = df['vert_velo'].values.tolist()

vert_velo_resp_list = df['vert_velo_resp'].values.tolist()


t1_list = df['t1'].values.tolist()
t2_list = df['t2'].values.tolist()
t3_list = df['t3'].values.tolist()
t4_list = df['t4'].values.tolist()


figure, axis = plt.subplots(6, 1)

axis[0].set_title('vert_velo')
axis[1].set_title('vert_velo_resp')
axis[2].set_title('t1')
axis[3].set_title('t2')
axis[4].set_title('t3')
axis[5].set_title('t4')

axis[0].plot(index, vert_velo_list, color = 'orange')
axis[1].plot(index, vert_velo_resp_list, color = 'blue')
axis[2].plot(index, t1_list, color = 'green')
axis[3].plot(index, t2_list, color = 'green')
axis[4].plot(index, t3_list, color = 'green')
axis[5].plot(index, t4_list, color = 'green')

figure.tight_layout(pad=5)

plt.legend(fontsize = '10')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




