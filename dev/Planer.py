import numpy as np 
import matplotlib.pyplot as plt  


class kCalculator:
    def ik():
        return 0
    def fk():
        return 0
    def jacobian():
        return 0
    def isf():
        return 0

class planer:
    def __init__(self,_k:kCalculator):
        self.k=_k
    def getPos(self,i:int):
        return 0
    def getJointAng(self,i:int):
        return 0
    
class finger_link(kCalculator): # 带连杆的3R机械臂
    A=np.mat([[1,0,0],[0,1,0],[0,-1,-1]])
    def __init__(self,_arg) -> None:
        super().__init__()
        self.arg=_arg
        self.RRR=finger_RRR(_arg)
    def fk(self,q): # q : 1*3
        q=np.reshape(q,[1,3])
        q=q+self.arg[0,3:6] # 实际值+初始bias
        pos=self.RRR.fk(q)
        return pos
    def ik(self,px): # px : 1*3
        px=np.reshape(px,[1,3])
        q=self.RRR.ik(px)-self.arg[0,3:6]
        return q
    def jacobian(self,q):
        q=np.reshape(q,[1,3])
        q=q-self.arg[0,3:6]

        q=np.reshape(q,[3,1]) # 开始矩阵运算 q1: 3*1
        q2=(np.linalg.inv(self.A)@q) # q2: 3*1
        J=self.RRR.jacobian(q2)@np.linalg.inv(self.A)
        return J
    def isk(self,q,F):
        q=np.reshape(q,[3,1])
        F=np.reshape(F,[3,1])
        tao=self.jacobian(q).T@F
        tao=np.reshape(tao,[1,3])
        return tao

class finger_RRR(kCalculator): # 经典3R机械臂
    def __init__(self,_arg) -> None:
        super().__init__()
        self.arg=_arg
    def fk(self,q):
        q=np.reshape(q,[1,3])
        l1=self.arg[0,0]
        l2=self.arg[0,1]
        l3=self.arg[0,2]
        r1=q[0,0]
        r2=q[0,1]
        r3=q[0,2]

        x=(l2*np.sin(r2)+l3*np.sin(r2+r3))*np.cos(r1)
        y=(l2*np.sin(r2)+l3*np.sin(r2+r3))*np.sin(r1)
        z= l2*np.cos(r2)+l3*np.cos(r2+r3)

        pos=np.mat([x,y,z])
        return pos

    def ik(self,px):
        px=np.reshape(px,[1,3])
        l1=self.arg[0,0]
        l2=self.arg[0,1]
        l3=self.arg[0,2]
        x=px[0,0]
        y=px[0,1]
        z=px[0,2]

        A=np.sqrt(x*x+y*y+z*z)
        k=np.sqrt(x*x+y*y)

        if x>=0:
            r1=np.arctan2(y,x)
            r2=np.pi/2-np.arctan2(z,k)-np.arccos((l2*l2+A*A-l3*l3)/(2*l2*A))
            r3=np.pi-np.arccos((l2*l2+l3*l3-A*A)/(2*l2*l3))
        else:
            r1=np.arctan2(y,x)
            if r1>=0:
                r1=r1-np.pi
            else:
                r1=r1+np.pi
            r2=np.arctan2(z,k)-np.pi/2-np.arccos((l2*l2+A*A-l3*l3)/(2*l2*A))
            r3=np.pi-np.arccos((l2*l2+l3*l3-A*A)/(2*l2*l3))

        q=np.mat([r1,r2,r3])
        return q
    def jacobian(self,q):
        q=np.reshape(q,[1,3])
        l1=self.arg[0,0]
        l2=self.arg[0,1]
        l3=self.arg[0,2]
        q1=q[0,0]
        q2=q[0,1]
        q3=q[0,2]   

        j11=(l2*np.sin(q2)+l3*np.sin(q2+q3))*(-np.sin(q1))
        j12=l2*np.cos(q1)*np.cos(q2)+l3*np.cos(q1)*np.cos(q2+q3)
        j13=l3*np.cos(q1)*np.cos(q2+q3)
        j21=(l2*np.sin(q2)+l3*np.sin(q2+q3))*np.cos(q1)
        j22=l2*np.sin(q1)*np.cos(q2)+l3*np.sin(q1)*np.cos(q2+q3)
        j23=l3*np.sin(q1)*np.cos(q2+q3)
        j31=0
        j32=-l2*np.sin(q2)-l3*np.sin(q2+q3)
        j33=-l3*np.sin(q2+q3)
        J=np.mat([[j11,j12,j13],[j21,j22,j23],[j31,j32,j33]])
        return J
    def isk(self,q,F):
        q=np.reshape(q,[3,1])
        F=np.reshape(F,[3,1])
        tao=self.jacobian(q).T@F
        tao=np.reshape(tao,[1,3])
        return tao


    


class testik(kCalculator):
    def __init__(self) -> None:
        super().__init__()
    def ik(self,px):
        return px


class testlineplaner(planer):
    def __init__(self, _k: kCalculator,_n:int=10):
        super().__init__(_k)
        self.n=_n
    def getPos(self,i:int):
        n=self.n
        xx=50*np.ones([1,n])
        yy=np.mat(np.linspace(-100,100,n))
        zz=-30*np.ones([1,n])

        if i>0 and i<=n:
            pos=np.mat([xx[0,i-1],yy[0,i-1],zz[0,i-1]])
        elif i<=0:
            pos=np.mat([0,0,0])
        else: # i>n
            pos=np.mat([xx[0,n-1],yy[0,n-1],zz[0,n-1]])
        return pos
    def getJointAng(self, i: int):
        if i>0 and i<=self.n:
            q=self.k.ik(self.getPos(i))
        elif i<=0:
            q=np.mat([0,0,0])
        else: # i>n
            q=self.k.ik(self.getPos(self.n))
        return q

class arrayplaner(planer):
    def __init__(self, _k: kCalculator,_arr:np.matrix):
        self.k=_k
        self.arr=_arr
        self.r,self.n=self.arr.shape
    def getPos(self, i: int):
        if i>0 and i<=self.n:
            pos=self.arr[:,i-1].T
        elif i<=0:
            pos=np.mat([0,0,0])
        else: # i>n
            pos=self.arr[:,self.n-1].T
        return pos
    def getJointAng(self, i: int):
        if i>0 and i<=self.n:
            q=self.k.ik(self.getPos(i))
        elif i<=0:
            q=np.mat([0,0,0])
        else: # i>n
            q=self.k.ik(self.getPos(self.n))
        return q

class resetplaner(planer):
    def __init__(self, _q:np.matrix,_n:int):
        self.q=_q
        self.n=_n
    def getJointAng(self, i: int):
        if i>0 and i<=self.n:
            q=(self.q-np.mat([0,0,0]))/self.n*i
        elif i<=0:
            q=np.mat([0,0,0])
        else: # i>n
            q=self.q
        return q


def agree(p1,p2):
    b=1
    for i in range(3):
        if np.abs(p1[0,i]-p2[0,i])>1e-6: #值不相等
            b=0
            break
    return b    # b=0 for not agree; b!=0 for agree

## 测试复位planer 
# rpll=resetplaner(np.mat([0.5,0.5,0.5]),10)
# for i in range(12):
#     print(rpll.getJointAng(i))

## =====================================================================

# ## 机器人数据
# arg=np.mat([0,100,100,0,0,0])
# px=np.mat([198,0,0])
# ## =====================================================================
# ## 测试直线plr，用v2的正逆运动学
# k1=finger_link(arg) #测v1
# k=finger_RRR(arg)
# # print(k.ik(px))
# # print(k.fk(k.ik(px)))
# lplr=testlineplaner(k,10)
# for i in range(12):
#     # q=lplr.getJointAng(i)
#     # print(q)
#     # j=k.jacobian(q)   #测v2的jacob
#     # print(j)          #测v2的jacob
#     # jj=k1.jacobian(q) #测v1的jacob
#     # print(jj)         #测v1的jacob

#     # ## 测试v2正运动学
#     # if agree(lplr.getPos(i),k.fk(lplr.getJointAng(i)))==0:
#     #     print(i)
#     #     print(lplr.getJointAng(i))
#     #     print(k.fk(lplr.getJointAng(i)))
## =====================================================================
# ## 测试直线plr，用v1的正逆运动学
# k=finger_link(arg)
# # print(k.ik(px))
# # print(k.fk(k.ik(px)))
# lplr=testlineplaner(k,10)
# for i in range(12):
#     print(lplr.getJointAng(i))

#     # ## 测试v1正运动学
#     # if agree(lplr.getPos(i),k.fk(lplr.getJointAng(i)))==0:
#     #     print(i)
#     #     print(lplr.getPos(i))
#     #     print(k.fk(lplr.getJointAng(i)))
## =====================================================================
## 测试输入数组plr
# n=10
# xx=50*np.ones([1,n])
# yy=np.mat(np.linspace(-100,100,n))
# zz=-30*np.ones([1,n])
# pp=np.append(np.append(xx,yy,0),zz,0)

# pll=arrayplaner(finger_RRR(arg),pp)
# for i in range(12):
#     print(pll.getJointAng(i))





