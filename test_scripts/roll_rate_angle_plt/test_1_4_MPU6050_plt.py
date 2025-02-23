import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('roll_measurement_test.csv')
print(df.head)
index = list(df.index.values)
rate_list = df['rate'].values.tolist()
angle_list = df['angle'].values.tolist()


figure, axis = plt.subplots(2, 1)

axis[0].plot(index, rate_list, label = 'rate', color = 'orange')
axis[1].plot(index, angle_list, label = 'angle')


plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()



