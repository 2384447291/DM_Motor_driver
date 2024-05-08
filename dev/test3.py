import Planer
import numpy as np

arg = np.mat([0,0.100,0.092,0,0,-23.58/180*np.pi]) 
px=np.mat([149/1000,0,0])
# k=Planer.finger_RRR(arg)
k=Planer.finger_link(arg)

# ik
q=k.ik(px)
print(q)

# fk
pos=k.fk(q)
print(pos)

# J
J=k.jacobian(q)
print(J)

F=3*np.mat([0,0,1])
# isk
tao=k.isk(q,F)
print(tao)

# fsk
ff=k.fsk(q,tao)
print(ff)