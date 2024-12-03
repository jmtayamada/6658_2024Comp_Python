import numpy as np
import matplotlib.pyplot as plt
from math import pi

import csv

# time, voltage, velocity

with open('dynamicBackward.csv', newline="") as f:
    reader = csv.reader(f)
    data = list(reader)

xs = [-float(i) for i in data[2]]
ys = [float(i) for i in data[1]]

slope, intercept = np.polyfit(xs, ys, 1)
print("kv = " + str(slope))
print("intercept = " + str(intercept))

plt.plot(xs, ys)
plt.plot(np.unique(xs), np.poly1d(np.polyfit(xs, ys, 1))(np.unique(xs)))
plt.show()
plt.close