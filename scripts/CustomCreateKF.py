from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise, dot3
import numpy as np
def createPersonKF(x,y, dt, dt2, KF_pd = .5,  KF_pv = .20,  KF_pa = .50,  KF_rd = 0.01, KF_rv = 0.1, KF_ra = 1, KF_q = 0.7):
	kalman_filter = KalmanFilter(dim_x=6, dim_z=2)
	# kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
	# dt = .1
	# dt2= .005
	# KF_F = np.array([[1., dt, 0 ,  0],
		# [0 , 1., 0 ,  0],
		# [0 , 0 , 1., dt],
		# [0 , 0 , 0 , 1.]])
	KF_Fca = np.array([[1.,dt,dt2/2],[0,1.,dt],[0,0,1]])
	KF_F = np.vstack((np.hstack((KF_Fca, np.zeros((3,3)) )),# np.zeros((3,3)))),
		np.hstack((np.zeros((3,3)), KF_Fca))  )) # , np.zeros((3,3)))) )))#,
	# np.hstack((np.zeros((3,3)),np.zeros((3,3)), KF_Fca))))
	# KF_q = 0.7 #0.3
	# KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=0.1, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=0.1, var=KF_q)))))
	KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(3, dt=0.1, var=KF_q),np.zeros((3,3)))),np.hstack((np.zeros((3,3)),Q_discrete_white_noise(3, dt=0.1, var=KF_q)))))
	# KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(3, dt=dt, var=KF_q),np.zeros((3,3)))),np.hstack((np.zeros((3,3)),Q_discrete_white_noise(3, dt=dt, var=KF_q)))))
	# KF_pd = 25.
	# KF_pv = 10.
	# KF_pa = 30.
	KF_P = np.diag([KF_pd, KF_pv, KF_pa,KF_pd, KF_pv, KF_pa])
	# KF_rd = 0.01 #0.05
	# KF_rv = 0.1 #0.2 #0.5
	# KF_ra = 1 #2 #0.5
	KF_R = np.diag([KF_rd,KF_rd])
	# KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv])
	# KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv, KF_ra, KF_ra])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0]])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]])
	KF_H = np.array([[1.,0,0,0,0,0],[0,0,0,1.,0,0]])

	# kalman_filter.x = np.array([x,0,y,0])
	kalman_filter.x = np.array([x,0,0,y,0,0])
	kalman_filter.F = KF_F
	kalman_filter.H = KF_H
	kalman_filter.Q = KF_Q
	kalman_filter.B = 0
	kalman_filter.R = KF_R
	kalman_filter.P = KF_P

	return kalman_filter

def createLegKF(x,y, dt, KF_pd = .5, KF_pv = .20, KF_rd = 0.01, KF_rv = 0.2, KF_q = 0.7):
	kalman_filter = KalmanFilter(dim_x=4, dim_z=2)
	# kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
	# dt = .1
	KF_F = np.array([[1., dt, 0 ,  0],
		[0 , 1., 0 ,  0],
		[0 , 0 , 1., dt],
		[0 , 0 , 0 , 1.]])
	# KF_q = 0.7 #0.3
	KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=0.1, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=0.1, var=KF_q)))))
	# KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=dt, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=dt, var=KF_q)))))
	# KF_pd = 25.
	# KF_pv = 10.
	KF_P = np.diag([KF_pd, KF_pv,KF_pd, KF_pv])
	# KF_rd = 0.05
	# KF_rv = 0.2 #0.5
	KF_R = np.diag([KF_rd,KF_rd])
	# KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv])
	KF_H = np.array([[1.,0,0,0],[0,0,1.,0]])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]])

	kalman_filter.x = np.array([x,0,y,0])
	kalman_filter.F = KF_F
	kalman_filter.H = KF_H
	kalman_filter.Q = KF_Q
	kalman_filter.B = 0
	kalman_filter.R = KF_R
	kalman_filter.P = KF_P

	return kalman_filter

def squareMatrix(mat, fillconstant):
	nrow , ncolumn = len(mat), len(mat[0])
	newmat = mat
	if nrow < ncolumn:
		for i in range(ncolumn - nrow):
			newmat.append([fillconstant]*ncolumn)
	elif ncolumn > nrow:
		for i in range(nrow - ncolumn):
			for _m in newmat:
				_m.append(fillconstant)

	return newmat


