import numpy as np 
import matplotlib.pyplot as plt  


class kCalculator:
    def ik():
        return 0
    def fk():
        return 0

class planer:
    def __init__(self,_k:kCalculator):
        self.k=_k
    def getPos(self,i:int):
        return 0
    def getJointAng(self,i:int):
        return 0
    
class fingerik(kCalculator):
    def __init__(self,_arg) -> None:
        super().__init__()
        self.arg=_arg
    def ik(self,px):
        j3bias=self.arg[0,5]
        l1=self.arg[0,0]
        l2=self.arg[0,1]
        l3=self.arg[0,2]
        x=px[0,0]
        y=px[0,1]
        z=px[0,2]

        A=np.sqrt(x*x+y*y)-l1
        B=-z
        r1=np.arctan2(y,x)*(180/np.pi)
        ccos=np.arccos((A*A+B*B+l2*l2-l3*l3)/(2*np.sqrt(A*A+B*B)*l2))*(180/np.pi)
        r2=ccos-(np.arctan2(B,A)*(180/np.pi))-90
        ccos2=np.arccos((-A*A-B*B+l2*l2+l3*l3)/(2*l2*l3))*(180/np.pi)
        if(ccos2>=90):
            ccos2=180-ccos2
        r3=ccos2-j3bias

        r3=-r2-r3
        q=np.mat([r1,r3,r2])/180*np.pi
        return q

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


# rpll=resetplaner(np.mat([0.5,0.5,0.5]),10)
# for i in range(12):
#     print(rpll.getJointAng(i))





# arg=np.mat([0,100,100,0,0,20.49])
# px=np.mat([100,0,0])

# # k=fingerik(arg)
# # print(k.ik(px))
# lplr=testlineplaner(fingerik(arg),10)
# for i in range(12):
#     print(lplr.getJointAng(i))


# n=10
# xx=50*np.ones([1,n])
# yy=np.mat(np.linspace(-100,100,n))
# zz=-30*np.ones([1,n])
# pp=np.append(np.append(xx,yy,0),zz,0)

# pll=arrayplaner(fingerik(arg),pp)
# for i in range(12):
#     print(pll.getJointAng(i))





