import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import math


def sigmoid(c1, x):
    z = 1 / (1 + np.exp(-c1 * x))
    return z


def fuzzy_model(error, a):

    if error <= (-a):
        y_value = -a
    elif (-a) < error < (-0.5 * a):
        y_value = error
    elif (-0.5 * a) < error < (0.5 * a):
        y_value = 0.7 * error
    elif (0.5 * a) < error < a:
        y_value = error
    else:
        y_value = a

    return y_value / a

a = 5
x = np.linspace(-a, a, 50)
y = [fuzzy_model(i, a*.5) for i in x]

plt.plot(x, y)
plt.show()
