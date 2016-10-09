%NS Qazi Umer Jamil
%DE 34 MTS B

%Please see the HW3_PID_Controller.m file first because that one is solved in
%very detail and step by step. 
%After building understanding from that problem, I solved it in direct
%approach without explaing much of the stuff.

close all;
clc;
clear;

% (2) - Lead-Lag Controller Design

%%System parameters:
M = 2.87;
m = 0.9;
b = 0.5;
I = 0.005;
g = 9.8;
l = 3.12;
Ts = 7;
Re = -4/Ts;
theeta = 0.005;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
%%System Transfer Function
%Source: http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling%
sys_tf = [P_cart ; P_pend];

% (2) - Lead-Lag Controller Design

rlocus(P_pend);
title('Root Locus of Plant (under Proportional Control)');

%%Continue rhe discussion AFTER implementing the PID Controller, the root
%%locus entirely shift to the left half plane if we place a pole at -0.2
%%and zeros at -1 -0.5. As below
z = [-1 -0.5];
p = -0.2;
k = 1;
C = zpk(z,p,k);
rlocus(C*P_pend)
title('Root Locus with Lead-Lag Controller Controller')

%To find the gain corresponding to a specific point on the root locus, we can use the rlocfind command. 
%Specifically, enter the command [k,poles] = rlocfind(C*P_pend) in the MATLAB command window.
%Then go to the plot and select a point on the root locus on left side of the loop, 
%Selecting these poles will ensure that the system settles sufficiently fast and, hopefully, that it has sufficient damping.

%[k,poles] = rlocfind(C*P_pend)


%selected_point =

 % -0.6905 + 0.0000i


%k =

 % 221.7485


%poles =

 %-23.7485 + 0.0000i
 % -0.6905 + 0.0000i
 % -0.6905 - 0.0000i
 %  0.0097 + 0.0000i

%Then we can check the impulse response of our closed-loop system to see if
%the requirements are met.

  K = 221.7485;
T = feedback(P_pend,K*C);
impulse(T)
title('Response of Pendulum Angle to an Impulse Disturbance under Lead Lag Control');

