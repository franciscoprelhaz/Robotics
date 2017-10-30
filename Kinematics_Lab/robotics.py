from os import write
from numpy import matrix, dot
from numpy.linalg import inv
from math import sqrt, pi, cos, sin, pow, atan2, asin, acos

# a, alpha, d, theta
#mixed variables missing

DOF0 = [0, -pi/2, 99]
DOF1 = [120, 0, 0]
DOF2_aux = [0, 0, 0]
DOF2 = [0, pi/2, 0]
DOF3 = [0, pi/2, 155]
DOF4 = [0, -pi/2, 0]
DOF5 = [0, 0, 0]


def A(a, alpha, d, theta):
	""" Matrix according to the DenHar Convention"""

	T_x = matrix([[1, 0, 0, a],
				[0, cos(alpha), -sin(alpha), 0],
				[0, sin(alpha), cos(alpha), 0],
				[0, 0, 0, 1]])
				
	T_z = matrix([[cos(theta), -sin(theta), 0, 0],
				[sin(theta), cos(theta), 0, 0],
				[0, 0, 1, d],
				[0, 0, 0, 1]])
	
	return dot(T_z, T_x)
	
	
def rob16b_dkinematics(theta0, theta1, theta2, theta3, theta4, theta5):
	""" Direct kinematics"""
	
	T = ( A(DOF0[0], DOF0[1], DOF0[2], theta0) * A(DOF1[0], DOF1[1], DOF1[2], theta1 - pi/2) *
		A(DOF2_aux[0], DOF2_aux[1], DOF2_aux[2], pi/2) * A(DOF2[0], DOF2[1], DOF2[2], theta2) *
		A(DOF3[0], DOF3[1], DOF3[2], theta3) * A(DOF4[0], DOF4[1], DOF4[2], theta4) *
		A(DOF5[0], DOF5[1], DOF5[2], theta5) )
	
	return T
		
		
def rob16b_ikinematics(T):
	""" Inverse kinematics"""
	
	P = T[0:3,3]
	R = T[0:3,0:3]
	
	# listing all constants to be used later
	c1 = sqrt( pow(P[0],2) + pow(P[1],2) )
	c2 = P[2] - DOF0[2]
	c3 = - ( pow(DOF3[2],2) - pow(c2,2) - pow(c1,2) - pow(DOF1[0],2) )/( 2*DOF1[0] )
	c_aux = sqrt( pow(c1,2) + pow(c2,2) )
	c4 = c1/c_aux
	c5 = c2/c_aux
	c6 = c3/c_aux
	if c4 == 0:
		c7 = 0
	else:
		c7 = atan2(c5,c4)
	
	# calculating thetas
	if P[0] <= abs(1e-14) and P[1] <= abs(1e-14):
		#singularity
		theta0 = 0
	elif P[0] <= abs(1e-14):
		# pi/2 or -pi/2
		theta0 = pi/2 #, -pi/2]
	else:
		theta0 = atan2(P[1], P[0]) #, atan2(P[1], P[0]) - pi]
		
	theta1 = asin(c6) - c7 #, pi - asin(c6) - c7]
	theta2 = acos( ( P[2] - DOF0[2] - DOF1[0]*cos(theta1) )/(DOF3[2]) ) - theta1 #,
			#-acos( ( P[2] - DOF0[2] - DOF1[0]*cos(theta1[0]) )/(DOF3[2]) ) - theta1[0]]
	
	A_0_3 = ( A(DOF0[0], DOF0[1], DOF0[2], theta0) * A(DOF1[0], DOF1[1], DOF1[2], theta1 - pi/2) *
		A(DOF2_aux[0], DOF2_aux[1], DOF2_aux[2], pi/2) * A(DOF2[0], DOF2[1], DOF2[2], theta2) )
		
	R_0_3 = A_0_3[0:3,0:3]
		
	r = inv(R_0_3) * R #R_3_6
	
	theta0 = round(theta0,3)
	theta1 = round(theta1,3)
	theta2 = round(theta2,3)
	theta3 = round(atan2(r[1,2], r[0,2]),3)
	theta4 =  round(atan2(sqrt( pow(r[0,2],2) + pow(r[1,2],2) ), r[2,2]),3)
	theta5 = round(atan2(-r[2,1], r[2,0]),3)
	
	return (theta0, theta1, theta2, theta3, theta4, theta5)
	

if __name__ == "__main__":
	try:
		while(True):

			print "Ok just write down some values for the thetas! We'll first apply the direct kinematics function and then the indirect!"
			print "Separate the values with a space, please!\n\n\n"
			input_data = raw_input()
			split_input = input_data.split(" ")
			if(len(split_input) == 6):
				q0 = float(split_input[0])
				q1 = float(split_input[1])
				q2 = float(split_input[2])
				q3 = float(split_input[3])
				q4 = float(split_input[4])
				q5 = float(split_input[5])
				
				print "Ok starting now!"
				answer_T = rob16b_dkinematics(q0, q1, q2,q3 , q4 ,q5) #put it in radians
				print "Direct Kinematics Answer:"
				print(answer_T)
				print "Inverse Kinematics Answer:"
				print(rob16b_ikinematics(answer_T))
			else:
				print "You didn't put in 6 args! Try again!"

				
	except KeyboardInterrupt, e:
		
		print "\nCTRL+C - Exiting user application."
		pass	
