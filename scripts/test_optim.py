from scipy.optimize import fmin_cg
from minimize import minimize
import numpy as np

args = (2, 3, 7, 8, 9, 10)

def f(x, *args):
	u, v = x
	a, b, c, d, e, f = args
	return a*u**2 + b*u*v + c*v**2 + d*u + e*v + f

def gradf(x, *args):
	u, v = x
	a, b, c, d, e, f = args
	gu = 2*a*u + b*v + d     # u-component of the gradient
	gv = b*u + 2*c*v + e     # v-component of the gradient
	return np.asarray((gu, gv))

x0 = np.asarray((0, 0))
res1 = fmin_cg(f, x0, fprime=gradf, args=args)
print res1
print minimize(x0, f, gradf, args, maxnumfuneval = 1000)
