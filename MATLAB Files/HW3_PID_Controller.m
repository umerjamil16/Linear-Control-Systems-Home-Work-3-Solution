close all;
clc;
clear;

% (1) - PID Controller Design

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


%%Here we are only interested in the control of the pendulum's position.
%Design requirements for this system are:
%1. Settling time for theta of less than 5 seconds
%2. Pendulum angle theta never more than 0.05 radians from the vertical
%3. Steady State error less then 3 sec

% (1) - PID Controller Design

rlocus(P_pend);
title('Root Locus of Plant (under Proportional Control)');

%As one of the branches of the above root locus is entirely in the right-half plane. 
%This means that will always be a closed-loop pole in the right-half plane, 
%thus making the system's impulse response unstable.
%To solve this problem, we need to add a pole at the origin (an integrator) 
%via the controller to cancel the plant zero at the origin. 
%This addition will produce two closed-loop poles in the right-half plane. 
%In our subsequent design we can then modify our controller to draw these poles into the left-half plane, 
%thereby stabilizing the closed-loop system. 
%So, adding a pole at origin, we get:

C = 1/s;
rlocus(C*P_pend)
title('Root Locus with Integral Control')

%So now we want to draw the root locus branches into the left-half plane to make the system stable. 
%Entering the following commands into the MATLAB command window will give us zeros and poles of our system.

zeros = zero(C*P_pend)
poles = pole(C*P_pend)

zeros = zero(C*P_pend)
poles = pole(C*P_pend)

%zeros =

 %    0


%poles =

%         0
%    2.0111
%   -2.0528
%   -0.1324
   
%So, there are four poles and only one zero. 
%This means that the root locus will have three asymptotes: 
%One along the real axis in the negative direction, and the other two at 120 degree angles to this one.
%This configuration is also unsatisfactory because we still have branches of the root locus that are entirely in the right-half complex plane. 
%In the above discussion we demonstrated that adding a zero to our integral controller 
%could pull the branches of the root locus to the left in the complex plane, 
%but we were not able to the pull the dominant branches far enough to the left. 
%A possible solution is to add yet another zero. 
%If we place both zeros on the negative real axis between the two plant poles, 
%then the two branches in the right-half plane will be pulled into the left-half plane 
%and will terminate at these two zeros. 
%Let's specifically evaluate the root locus for a controller with an
%integrator and zeros at -1, -0.5
%
z = [-1 -0.5];
p = 0;
k = 1;
C = zpk(z,p,k);
rlocus(C*P_pend)
title('Root Locus with PID Controller')

%Examining the above root locus helps us to determine whether or not our given requirements can be met. 
%Specifically, since it is desired that the settling time of the system be less than 7 seconds, 
%the real parts of our dominant closed-loop poles should be less than approximately -4/7 = -0.571. 
%In other words, our dominatnt closed-loop poles should be located in the complex  
% to the left of a vertical line at $s = -0.571$. 
%Since it is also desired that the pendulum not move more than 0.06 radians away from vertical, 
%we also want to ensure that the closed-loop system has sufficient damping. 
%Placing the dominant closed-loop poles near the real axis will increase the system's damping.

%To find the gain corresponding to a specific point on the root locus, we can use the rlocfind command. 
%Specifically, enter the command [k,poles] = rlocfind(C*P_pend) in the MATLAB command window.
%Then go to the plot and select a point on the root locus on left side of the loop, 
%Selecting these poles will ensure that the system settles sufficiently fast and, hopefully, that it has sufficient damping.

[k,poles] = rlocfind(C*P_pend)


%selected_point =

%  -0.7087 + 0.0011i


%k =

%  310.5155


%poles =

 %  0.0000 + 0.0000i
 %-33.4083 + 0.0000i
 % -0.7087 + 0.0011i
 % -0.7087 - 0.0011i
 
%Then we can check the impulse response of our closed-loop system to see if
%the requirements are met.
 K = 310.51;
T = feedback(P_pend,K*C);
impulse(T)
title('Response of Pendulum Angle to an Impulse Disturbance under PID Control');

 