import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
end_data=np.loadtxt("/home/mzc/codes/ROS/perception_ws/files/odom/odom_end.txt")
start_data=np.loadtxt("/home/mzc/codes/ROS/perception_ws/files/odom/odom_start.txt")
# f_end=open("/home/mzc/codes/ROS/perception_ws/files/odom/odom_end.txt")
# f_start=open("/home/mzc/codes/ROS/perception_ws/files/odom/odom_start.txt")
xe=end_data[...,0]
ye=end_data[...,1]
ze=end_data[...,2]
Re=end_data[...,3]
Pe=end_data[...,4]
Ye=end_data[...,5]
xe=xe-0.02
ye=ye-0.06
ze=ze+0.07
Re=Re-0.01
Ye=Ye-0.01
xs=start_data[...,0]
ys=start_data[...,1]
zs=start_data[...,2]
Rs=start_data[...,3]
Ps=start_data[...,4]
Ys=start_data[...,5]

xea=np.mean(xe)
yea=np.mean(ye)
zea=np.mean(ze)

Rea=np.mean(Re)
Pea=np.mean(Pe)
Yea=np.mean(Ye)

xsa=np.mean(xs)
ysa=np.mean(ys)
zsa=np.mean(zs)

Rsa=np.mean(Rs)
Psa=np.mean(Ps)
Ysa=np.mean(Ys)
fig=plt.figure()
ax=Axes3D(fig)
ax.scatter(xe,ye,ze,c='r',label="End")
ax.scatter(xea,yea,zea,c="green",label="center of End",s=150)
ax.scatter(xs,ys,zs,c='b',label="Start")
ax.scatter(xsa,ysa,zsa,c="yellow",label="center of Start",s=150)
plt.legend(loc='upper left')
ax.set_zlabel("Z")
ax.set_xlabel("Y")
ax.set_ylabel("X")
plt.show()

plt.subplot(2,1,1)
plt.plot(xe,color="red", linewidth=2.5, linestyle="-", label="x")
plt.plot(ye,color="blue", linewidth=2.5, linestyle="-", label="y")
plt.plot(ze,color="green", linewidth=2.5, linestyle="-", label="z")
plt.legend(loc='upper left')
plt.subplot(2,1,2)
plt.plot(Re,color="red", linewidth=2.5, linestyle="-", label="roll")
plt.plot(Pe,color="blue", linewidth=2.5, linestyle="-", label="pitch")
plt.plot(Ye,color="green", linewidth=2.5, linestyle="-", label="yaw")
# plt.plot(line,color="green", linewidth=2.0, linestyle=":", label="collision distance")
plt.legend(loc='upper left')
plt.show()

print(np.max(xe)-xea,np.max(ye)-yea,np.max(ze)-zea)
print(np.max(xe)-Rea,np.max(Pe)-Pea,np.max(Ye)-Yea)

print(xea-xsa,yea-ysa,zea-zsa)
print(Rea-Rsa,Pea-Psa,Yea-Ysa)

# mean_fs=np.mean(fs)
# mean_fs2=np.mean(fs2)
# ratio_dis=(mean_fs-mean_fs2)/mean_fs2;
# succfs=1-num_fs/len(fs)
# succfs2=1-num_fs2/len(fs2)
# print(mean_fs,succfs)
# print(mean_fs2,succfs2)
# print(ratio_dis)