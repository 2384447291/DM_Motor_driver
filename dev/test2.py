import numpy as np
import Planer

# limit=np.mat([3,0,0])
# fb=np.mat([2.61048799 , 0.       ,  30.0009354 ])

# print(Planer.overlimit(limit,fb))

n=100
n=int(n/4)*4
l=int(n/4)
zz=20*np.ones([1,n])/1000

x1=np.mat(np.linspace(50,150,l))/1000
x2=150*np.ones([1,l])/1000
x3=np.mat(np.linspace(150,50,l))/1000
x4=50*np.ones([1,l])/1000
xx=np.hstack([x1,x2,x3,x4])

y1=-50*np.ones([1,l])/1000
y2=np.mat(np.linspace(-50,50,l))/1000
y3=50*np.ones([1,l])/1000
y4=np.mat(np.linspace(50,-50,l))/1000
yy=np.hstack([y1,y2,y3,y4])

for i in range(n):
    print([xx[0,i],yy[0,i],zz[0,i]])
