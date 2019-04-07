#   This source codetr is part of the graph optimization package
#   deveoped for the lectures of robotics2 at the University of Freiburg.
#  
#     Copyright (c) 2007 Giorgio Grisetti, Gian Diego Tipaldi
#  
#   It is licences under the Common Creative License,
#   Attribution-NonCommercial-ShareAlike 3.0
#  
#   You are free:
#     - to Share - to copy, distribute and transmit the work
#     - to Remix - to adapt the work
#  
#   Under the following conditions:
#  
#     - Attribution. You must attribute the work in the manner specified
#       by the author or licensor (but not in any way that suggests that
#       they endorse you or your use of the work).
#    
#     - Noncommercial. You may not use this work for commercial purposes.
#    
#     - Share Alike. If you alter, transform, or build upon this work,
#       you may distribute the resulting work only under the same or
#       similar license to this one.
#  
#   Any of the above conditions can be waived if you get permission
#   from the copyright holder.  Nothing in this license impairs or
#   restricts the author's moral rights.
#  
#   This software is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied 
#   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#   PURPOSE.  


#the measurements are coming in the
#Z matrix

#this function solves the odometry calibration problem
#given a measurement matrix Z.
#Every row of the matrix contains
#z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
#Z:	The measurement matrix
#X:	the calubration matrix
#returns the bias correction matrix BIAS
function X=ls_calibrate_odometry(Z)
	#accumulator variables for the linear system
	H=zeros(9,9);
	b=zeros(9,1);
	#initial solution (the identity transformation)
	X=eye(3); 
	
	#loop through the measurements and update the
	#accumulators
	for i=1:size(Z,1),
		e=error_function(i,X,Z);
		A=jacobian(i,Z);
		H=H+A'*A;
		b=b+A'*e;
	end
	#solve the linear system
	deltaX=-H\b;
	#this reshapes the 9x1 increment vector in a 3x3 atrix
	dX=reshape(deltaX,3,3)';
	#computes the cumulative solution
	X=X+dX;
end

#this function computes the error of the i^th measurement in Z
#given the calibration parameters
#i:	the number of the measurement
#X:	the actual calibration parameters
#Z:	the measurement matrix
#e:	the error of the ith measurement
function e=error_function(i,X,Z)
	uprime=Z(i,1:3)';
	u=Z(i,4:6)';
	e=uprime-X*u;
end

#derivative of the error function for the ith measurement in Z
#does not depend on the state
#i:	the measuement number
#Z:	the measurement matrix
#A:	the jacobian of the ith measurement
function A=jacobian(i,Z)
	u=Z(i,4:6);
	A=zeros(3,9);
	A(1,1:3)=-u;
	A(2,4:6)=-u;
	A(3,7:9)=-u;
end
