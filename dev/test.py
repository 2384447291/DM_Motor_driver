import numpy as np
import Planer

# A=np.mat([[1,0,0],[0,1,0],[0,-1,-1]])
# B=np.mat([3,4,5])
# C=np.mat([[1],[2],[3]])

# d1=np.reshape(B,[3,1])
# d2=np.reshape(C,[3,1])

# print(d1)
# print(d2)
# print(B[0,1:3].T)


A=np.mat([1,0,0])
B=np.mat([3,4,5])
C=np.mat([[-1],[2],[3]])


limit=-1*np.mat([3,4,5])
feedback=np.mat([[-1],[2],[3]])

limit=np.reshape(limit,[3,1])
feedback=np.reshape(feedback,[3,1])

print(Planer.overlimit(-1*C,B))
print(limit[0,0]*feedback[0,0]+limit[1,0]*feedback[1,0]+limit[2,0]*feedback[2,0])
print(feedback.T@limit>=0)



