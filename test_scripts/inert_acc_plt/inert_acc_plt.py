import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('inert_acc_test.csv')
print(df.head)
index = list(df.index.values)
acc_list = df['inert_acc'].values.tolist()

x_list = [0, 500, 1000]
y_list = [0, 0, 0]

plt.plot(x_list, y_list, linestyle='dashed', color='gray')
plt.plot(index, acc_list, label = 'inert_acc', color = 'blue')

plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measurement', fontsize="15")
plt.show()




