more off;
#load the calibration matrix
disp('loading the matrix');
Z=load('data/calib.dat');

#compute the ground truth trajectory
TrueTrajectory=compute_odometry_trajectory(Z(:,1:3));
hold on;
disp('ground truth');
plot(TrueTrajectory(:,1),TrueTrajectory(:,2));
pause(1);

#compute the uncalibrated odometry
OdomTrajectory=compute_odometry_trajectory(Z(:,4:6));
hold on;
disp('odometry');
plot(OdomTrajectory(:,1),OdomTrajectory(:,2));
pause(1);

disp('computing calibration parameters');
#compute the calibration parameters
X=ls_calibrate_odometry(Z);
hold on;
disp(X);
pause(1);

disp('computing calibrated odometry');
COdom=apply_odometry_correction(X,Z(:,4:6));
CalTrajectory=compute_odometry_trajectory(COdom);
plot(CalTrajectory(:,1),CalTrajectory(:,2));
