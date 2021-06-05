import numpy as np
import math
import sys

x = sys.argv[1]
y = sys.argv[2]
z = sys.argv[3]

h = float(z) / np.sqrt(float(x)**2 + float(y)**2)
angle = math.atan(h)
print(angle * 180 / 3.14159265358)