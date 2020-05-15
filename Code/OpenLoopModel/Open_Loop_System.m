% Chi Zhang, Shakshi Himmatranka, Sumanyu Singh
% GM Capstone - Electrified Driveline System Modelling
% Date of Creation Nov 14th, 2019
% University of California - Berkeley

% DO NOT CIRCULATE.

%% 
clear all; close all; clc

% % Get linearized Matrices
Tbrake = 0;
Tslope = 0;
VehicleSpeed = 20; % mph 
[LinearMatrix, EqPoints] = TaylorLinearization(VehicleSpeed,Tslope,Tbrake);

% State Space Matrices
A = double(LinearMatrix(1:5,1:5));
%B = double(LinearMatrix(1:5,6:7));
B = 10*double(LinearMatrix(1:5,6:8));
% For Outputs: assign C matrix to the output variables only and D matrix to
% row (value = #outputs), column (value = #col of B matrix)

C = [1 0 0 0 0;
     0 1 0 0 0];

D = zeros(size(C,1),size(B,2));

disp('Variables and Matrices Created ...')
%% Continuous Time Model

% State Space Creation

vehicle = ss(A,B,C,D, 'InputName',{'Tm','Tslope','Tbrake'},'OutputName',...
    {'Motor Speed','Wheel Speed'},'StateName',...
    {'Motor Speed','Wheel Speed','Vehicle Speed','Shaft Torque',...
    'Suspension Torque'});
vehicle.TimeUnit = 'seconds';

%% Discrete Time Model

% State Space Creation
Tsampling = 0.001;
vehicleDiscrete = c2d(vehicle,Tsampling); %default is Zero-Order Hold
vehicleDiscrete.TimeUnit = 'seconds';

%% Analysis - Continuous Time vs Discrete Time

% Continous Time
eigenvalues = eig(vehicle);
polevalues = pole(vehicle);
stability = isstable(vehicle);

if stability == 1
    disp('Stable for Continous')
else
    disp('Unstable for Continous')
end

% Discrete Time
eigenvaluesD = eig(vehicleDiscrete);
polevaluesD = pole(vehicleDiscrete);
stabilityD = isstable(vehicleDiscrete);

if stabilityD == 1
    disp('Stable for Discrete')
else
    disp('Unstable for Discrete')
end

vehicletf = tf(vehicleDiscrete);

options = bodeoptions;
% options.xlim = [10^-8,10^3];
options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.

% Plots
% figure(1)
% step(vehicletf,10000)
figure(2)
bode(vehicletf(1,1),options)
% figure(3)
% impulse(vehicletf)

