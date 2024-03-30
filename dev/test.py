import numpy as np

A=np.mat([[1,0,0],[0,1,0],[0,-1,-1]])
B=np.mat([3,4,5])
C=np.mat([[1],[2],[3]])

d1=np.reshape(B,[3,1])
d2=np.reshape(C,[3,1])

print(d1)
print(d2)
print(B[0,1:3].T)





