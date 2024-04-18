import numpy as np
import Planer

limit=np.mat([3,0,0])
fb=np.mat([2.61048799 , 0.       ,  30.0009354 ])

print(Planer.overlimit(limit,fb))