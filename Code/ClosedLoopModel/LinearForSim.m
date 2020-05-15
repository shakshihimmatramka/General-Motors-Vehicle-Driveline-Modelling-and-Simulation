function [LinearMatrix, EqPoints] = LinearForSim(VehicleSpeed,Tbrake,Tslope)
m = 3875*0.45359; % Vehicle Curb Weight in kg
r = 1 + 350/60; % Gear ratio
Jm = 0.038; % Moment of Inertia for Motor
ks = 120*57.3; % Half Shaft Stiffness in Nm/rad
bs = 10; % Half Shaft Damping in Nm/(rad*sec)
Jw = 4.0; % Moment of Inertia for Wheel Assembly
ksus = 700*57.3; % Suspension Stiffness in Nm/rad
bsus = 220; % Suspension Damping in Nm/(rad*sec)
rw = 0.32; % Wheel Radius
g = 9.81; % Acceleration due to gravity
% c1,c2,c3 are the coefficients for the friction coefficient calculation
c1 = 1.2801;
c2 = 23.99;
c3 = 0.52;
cd = 0.35; % Drag Coefficient
rho = 1.275; % Air Density (kg/m^3)
A = 1.75*1.56; % Cross-Sectional Area for Chevy Bolt m^2
vs = VehicleSpeed;
Tb = Tbrake;
Tslp = Tslope;
%% Define states and inputs which we need to solve 
syms ms ws Ts Tsus Tm %(5 variables for 5 equations)
%% System of Equations
lambda = (ws-vs)/ws;
miu = c1*(1-exp(-c2*lambda))-c3*lambda;
eq1 = (Tm-1/r*(Ts+bs*(ms/r-ws)))/Jm;
eq2 = (Ts+bs*(ms/r-ws)-Tsus-bsus*(ws-vs)-m*g*rw*miu-Tb)/Jw;
eq3 = (m*g*rw*miu+Tsus+bsus*(ws-vs)-Tslp-0.5*rho*(vs*rw)^2*cd*A*rw)/(m*rw^2);
eq4 = ks*(ms/r-ws);
eq5 = ksus*(ws-vs);
equations = [eq1,eq2,eq3,eq4,eq5];
vars = [ms, ws, Ts, Tsus, Tm];
%% Find equlibrium points
eq = solve(equations==0,vars);
eqms = eq.ms;
eqws = eq.ws;
eqvs = vs;
eqTs = eq.Ts;
eqTsus = eq.Tsus;
eqTm = eq.Tm;
eqTslp = Tslp;
eqTb = Tb;
equlibrium = [eqms,eqws,eqvs,eqTs,eqTsus,eqTm,eqTslp,eqTb];
EqPoints = double(equlibrium)';
%% Redefine all the states and Inputs
syms ms ws vs Ts Tsus Tm Tslp Tb
%% Taylor series
% Redefine equations and variables to have them all in symbolic form
lambda = (ws-vs)/ws;
miu = c1*(1-exp(-c2*lambda))-c3*lambda;
eq1 = (Tm-1/r*(Ts+bs*(ms/r-ws)))/Jm;
eq2 = (Ts+bs*(ms/r-ws)-Tsus-bsus*(ws-vs)-m*g*rw*miu-Tb)/Jw;
eq3 = (m*g*rw*miu+Tsus+bsus*(ws-vs)-Tslp-0.5*rho*(vs*rw)^2*cd*A*rw)/(m*rw^2);
eq4 = ks*(ms/r-ws);
eq5 = ksus*(ws-vs);
equations = [eq1,eq2,eq3,eq4,eq5];
variables = [ms, ws, vs, Ts, Tsus, Tm, Tslp,Tb];
jacob = jacobian(equations, variables);
for i =1:5 % number of state
    for j =1:8 % number of states+ number of inputs
        LinearMatrix(i,j) = subs(jacob(i,j),variables,equlibrium);
    end
end
end