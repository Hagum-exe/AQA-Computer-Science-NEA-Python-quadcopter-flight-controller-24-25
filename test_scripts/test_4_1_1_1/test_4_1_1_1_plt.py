import pandas as pd
import matplotlib.pyplot as plt



df = pd.read_csv('calib_pre_filter.csv')
df2 = pd.read_csv('calib_post_filter.csv')

#df = pd.read_csv('dynmc_calib.csv')
#df2 = pd.read_csv('dynmc_post_filter.csv')

'''
index = list(df.index.values)
alt_list = df['alt'].values.tolist()
alt_list_post = df2['alt_post'].values.tolist()

plt.plot(index, alt_list, label = 'velo pre-filter', color='c')
plt.plot(index, alt_list_post, label = 'velo post-filter', color='orange')
'''

index = list(df.index.values)
velo_list = df['alt'].values.tolist()
velo_list_post = df2['alt_post'].values.tolist()


plt.plot(index, velo_list, label = 'velo pre-filter', color='c')
plt.plot(index, velo_list_post, label = 'velo post-filter', color='orange')



plt.legend(fontsize = '15')
plt.xlabel('Sample number (indexed by time)', fontsize="15")
plt.ylabel('Measured vertical velocity', fontsize="15")
print(df.std())
print(df2.std())
plt.show()
 



