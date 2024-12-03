import numpy as np
import matplotlib.pyplot as plt
from math import pi
import pandas as pd

import csv

# time, voltage, velocity

dataframe = pd.read_csv("dynamicForward.csv")

xs = dataframe.iloc[:, 2]
ys = dataframe.iloc[:, 1]

slope, intercept = np.polyfit(xs, ys, 1)
print("kv = " + str(slope))
print("intercept = " + str(intercept))

plt.plot(xs, ys)
plt.plot(np.unique(xs), np.poly1d(np.polyfit(xs, ys, 1))(np.unique(xs)))
plt.show()
plt.close