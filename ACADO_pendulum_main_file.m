% Optimal path for a pendulum using ACADO
% The code is written by: Amin Ghorbanpour 2/20/2021

clear all;close all;clc;
 global g M l Io mo gr b Lm P R Vs m lG
%% DATA

g = 9.81;
M = 0.326/2;   % mass at the end of pendulum
m = 0.317;
l = 0.355;  % length of pendulum
Io = (m/3 + M)*l^2;   % pendulum inertia
lG = (M*l + 0.5*m*l)/(M+m);

    % Motor
mo = 0.00001;     % moment of inertia of motor
gr = 7.5;        % gear ratio
b = 0.00001;    % friction coefficient
Lm = 0.0458;      % back emf constant lambda
P = 4;          % motor poles
R = 1;  % resistant
Vs = 10;    % capacitor voltage

%%
BEGIN_ACADO;                                % Always start with "BEGIN_ACADO".
   
    acadoSet('problemname', 'pendulum_OPT_acado');     % Set your problemname. If you
                                                    % skip this, all files will
                                                    % be named "myAcadoProblem"
   
    DifferentialState       q qdot Cost;              % The differential states
    Control                 ra rb rc tau;              % The controls
%%
f = acado.DifferentialEquation();       % Set the differential equation object
    f.linkMatlabODE('pendulum_dyn')
   
   
    ocp = acado.OCP(0.0, 1, 20 );          % Set up the Optimal Control Problem (OCP)
    ocp.minimizeMayerTerm(Cost);
   
   
    ocp.subjectTo( f );
   
    ocp.subjectTo( 'AT_START', q == pi-5*pi/180);
    ocp.subjectTo( 'AT_START', qdot == 0 );
    ocp.subjectTo( 'AT_START', Cost == 100);
   
    ocp.subjectTo( 'AT_END', q == 0 );
    ocp.subjectTo( 'AT_END', qdot == 0.0 );
   
    ocp.subjectTo(  -1 <= ra <= 1 );
    ocp.subjectTo(  -1 <= rb <= 1 );
    ocp.subjectTo(  -1 <= rc <= 1 );
   
    ocp.subjectTo(   0 <= tau - ((Lm*gr*Vs)/R)*(sin(P*gr*q/2)*ra + sin((P*gr*q/2) + 2*pi/3)*rb + sin((P*gr*q/2) - 2*pi/3)*rc) <= 0 );
    ocp.subjectTo(   0 <= (ra+rb+rc) - ((Lm*gr*qdot)/Vs)*(sin(P*gr*q/2) + sin((P*gr*q/2) + 2*pi/3) + sin((P*gr*q/2) - 2*pi/3)) <= 0 );


   
 %%  
    algo = acado.OptimizationAlgorithm(ocp);
    algo.set('INTEGRATOR_TOLERANCE', 1e-4 ); % Set some parameters for the algorithm
    algo.set( 'KKT_TOLERANCE', 1e-5 );
   
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m.
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
 
 out = pendulum_OPT_acado_RUN();                % Run the test  