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

#computes a calibrated vector of odometry measurements
#by applying the bias term to each line of the measurements
#X: 	3x3 matrix obtained by the calibration process
#U: 	Nx3 matrix containing the odometry measurements
#C:	Nx3 matrix containing the corrected odometry measurements	

function C=apply_odometry_correction(X, U)
	C=zeros(size(U,1),3);
	for i=1:size(U,1),
		u=U(i,1:3)';
		uc=X*u;
		C(i,:)=uc;
	end
end