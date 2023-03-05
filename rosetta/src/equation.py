#! /usr/bin/python3
from numpy import arange
from pandas import read_csv
from scipy.optimize import curve_fit
from matplotlib import pyplot

# define the true objective function
def objective(x, a, b, c, d, e, f):
	return a * x**5 + b * x**4 + c * x**3 + d * x**2 + e * x + f

# load the dataset
dataframe = read_csv('/home/dafodilrat/Downloads/sach_is_great.csv')
x = dataframe['pwm']
y = dataframe['rpm']
# curve fit
popt, _ = curve_fit(objective, x, y)
# summarize the parameter values
a, b, c, d, e, f = popt
s = 1100
print(f+ e*s + d*s**2 + c*s**3 + b*s**4 + a*s**5)
print(a,b,c,d,e,f)
# plot input vs output
pyplot.scatter(x, y)
# define a sequence of inputs between the smallest and largest known inputs
x_line = arange(min(x), max(x), 1)
# calculate the output for the range
y_line = objective(x_line, a, b, c, d, e, f)
# create a line plot for the mapping function
pyplot.plot(x_line, y_line, '--', color='red')
pyplot.show()

