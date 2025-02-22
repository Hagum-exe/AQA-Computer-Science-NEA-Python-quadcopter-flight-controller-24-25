import pandas as pd
import matplotlib.pyplot as plt



#df = pd.read_csv('calib_pre_filter.csv')
#df2 = pd.read_csv('calib_post_filter.csv')

df = pd.read_csv('alt_test.csv')

index = list(df.index.values)
alt_list = df['alt'].values.tolist()


plt.plot(index, alt_list, label = 'alt pre-filter')




plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measured altitude', fontsize="15")
plt.show()




