function [AD,BD] = StateSpace(LinearMatrix)

A = double(LinearMatrix(1:5,1:5));
%B = double(LinearMatrix(1:5,6:7));
B = 10*double(LinearMatrix(1:5,6:8));
% For Outputs: assign C matrix to the output variables only and D matrix to
% row (value = #outputs), column (value = #col of B matrix)
C = [1 0 0 0 0;
     0 1 0 0 0];
D = zeros(size(C,1),size(B,2));

vehicle = ss(A,B,C,D, 'InputName',{'Tm','Tslope','Tbrake'},'OutputName',...
    {'Motor Speed','Wheel Speed'},'StateName',...
    {'Motor Speed','Wheel Speed','Vehicle Speed','Shaft Torque',...
    'Suspension Torque'});
vehicle.TimeUnit = 'seconds';

Tsampling = 0.001;
vehicleDiscrete = c2d(vehicle,Tsampling); %default is Zero-Order Hold
vehicleDiscrete.TimeUnit = 'seconds';
AD = vehicleDiscrete.A;
BD = vehicleDiscrete.B;
% CD = vehicleDiscrete.C;
% DD = vehicleDiscrete.D;
end