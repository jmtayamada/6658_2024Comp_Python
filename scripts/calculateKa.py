import numpy as np
import matplotlib.pyplot as plt
from math import pi

import csv

# time, voltage, velocity

with open('quasitastic.csv', newline="") as f:
    reader = csv.reader(f)
    data = list(reader)

xs = [-float(i) for i in data[0]]
ys = [float(i) for i in data[2]]

slope, intercept = np.polyfit(xs, ys, 1)
print("kv = " + str(slope))

plt.plot(xs, ys)
plt.plot(np.unique(xs), np.poly1d(np.polyfit(xs, ys, 1))(np.unique(xs)))
plt.show()
plt.close