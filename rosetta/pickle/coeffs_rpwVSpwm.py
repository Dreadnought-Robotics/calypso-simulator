import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import math
import pickle

data = pd.read_csv(r'/home/sacchin/Desktop/haroon/RPMvsPWM.csv')
x = data['pwm']
y = data['rpm']

def objective(x, e, f, g, h):
    return x**3*e + x**2*f + x*g + h

popt, _ = curve_fit(objective, x, y)
e, f, g, h = popt

print('e = %.5f , f = %.5f , g = %.5f , h = %.5f' % (e, f, g, h))
# print(e,f,g,h)
# xx = 1800
# print(xx**3*e + xx**2*f + xx*g + h)

plt.scatter(x,y)

x_line = np.arange(min(x), max(x), 1)
y_line = objective(x_line, e, f, g, h)

plt.plot(x_line, y_line, '--', color='red')
plt.show()

coeff_list = [e,f,g,h]
pickle.dump(coeff_list, open('/home/sacchin/calypso_ws/src/pickle/coeffs.pickle', 'wb'))