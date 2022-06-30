import numpy as np
import matplotlib.pyplot as plt

fs=np.loadtxt("/home/mzc/Documents/log/save_my2/fs_dis.txt")
fs2=np.loadtxt("/home/mzc/Documents/log/save_comp/fs_dis.txt")
line=np.ones(len(fs)+10)
line=line*0.2
fs+=0.15
plt.plot(fs,color="red", linewidth=2.5, linestyle="-", label="test group")
plt.plot(fs2,color="blue", linewidth=2.5, linestyle="-", label="control group")
plt.plot(line,color="green", linewidth=2.0, linestyle=":", label="collision distance")
plt.legend(loc='upper left')
plt.show()
num_fs=0
num_fs2=0
for dis in fs:
    if dis<0.2:
        num_fs+=1
for dis in fs2:
    if dis<0.2:
        num_fs2+=1
mean_fs=np.mean(fs)
mean_fs2=np.mean(fs2)
ratio_dis=(mean_fs-mean_fs2)/mean_fs2;
succfs=1-num_fs/len(fs)
succfs2=1-num_fs2/len(fs2)
print(mean_fs,succfs)
print(mean_fs2,succfs2)
print(ratio_dis)