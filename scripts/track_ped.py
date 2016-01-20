#!/usr/bin/env python
from __future__ import division, print_function
import rospy

from book_format import load_style
load_style()

from KalmanFilter import *

from numpy import *
from numpy.linalg import inv
#time step of mobile movement
dt = 0.1
# Initialization of state matrices
X = array([[0.0], [0.0], [0.1], [0.1]])
P = diag((0.01, 0.01, 0.01, 0.01))
A = array([[1, 0, dt , 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
Q = eye(X.shape[0])
B = eye(X.shape[0])
U = zeros((X.shape[0],1)) 

# Measurement matrices
Y = array([[X[0,0] + abs(random.randn(1)[0])], [X[1,0] + abs(random.randn(1)[0])]])
H = array([[1, 0, 0, 0], [0, 1, 0, 0]])
R = eye(Y.shape[0])
# Number of iterations in Kalman Filter
N_iter = 50
# Applying the Kalman Filter
for i in arange(0, N_iter):
	(X, P) = kf_predict(X, P, A, Q, B, U)
	(X, P, K, IM, IS, LH) = kf_update(X, P, Y, H, R)
	Y = array([[X[0,0] + abs(0.1 * random.randn(1)[0])],[X[1, 0] + abs(0.1 * random.randn(1)[0])]]) 
