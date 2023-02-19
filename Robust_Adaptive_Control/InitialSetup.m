syms a1 b1 c1 d1 a2 b2 c2 d2 real
syms t1_dot t2_dot t1 t2 real %t is time t1 is theta1 t2 is theta2

t0 = 0;
tf = 10;

eqn1 = a1*t0^3 + b1*t0^2 + c1*t0 + d1 == pi;
eqn2 = a2*t0^3 + b2*t0^2 + c2*t0 + d2 == pi/2;
eqn3 = 3*a1*t0^2 + 2*b1*t0 + c1 == 0;
eqn4 = 3*a2*t0^2 + 2*b2*t0 + c2 == 0;

eqn5 = a1*tf^3 + b1*tf^2 + c1*tf + d1 == 0;
eqn6 = a2*tf^3 + b2*tf^2 + c2*tf + d2 == 0;
eqn7 = 3*a1*tf^2 + 2*b1*tf + c1 == 0;
eqn8 = 3*a2*tf^2 + 2*b2*tf + c2 == 0;

[a1,b1,c1,d1]=solve([eqn1,eqn3,eqn5,eqn7],[a1,b1,c1,d1]);
[a2,b2,c2,d2]=solve([eqn2,eqn4,eqn6,eqn8],[a2,b2,c2,d2]);

t = 0:0.01:10;
%syms t;
t1_des = a1*t.^3 + b1*t.^2 + c1.*t + d1;
t2_des = a2*t.^3 + b2*t.^2 + c2.*t + d2;

t1_dot_des = 3*a1*t.^2 + 2*b1.*t + c1;
t2_dot_des = 3*a2*t.^2 + 2*b2.*t + c2;

t1_ddot_des = 6*a1.*t + 2*b1;
t2_ddot_des = 6*a2.*t + 2*b2;

plot(t,t1_des,'lineWidth',2.0);
hold on
plot(t,t2_des,'lineWidth',2.0);
xlabel('Time(sec)');
ylabel('Theta(rad)');
legend('Theta1','Theta2');
title('Theta v/s. Time');

figure
t1_dot_des = 3*a1*t.^2 + 2*b1.*t + c1;
t2_dot_des = 3*a2*t.^2 + 2*b2.*t + c2;
plot(t,t1_dot_des,'lineWidth',2.0);
hold on
plot(t,t2_dot_des,'lineWidth',2.0);
xlabel('Time(sec)');
ylabel('ThetaDot(rad/s)');
legend('Theta1Dot','Theta2Dot');
title('ThetaDot v/s. Time');

figure
t1_ddot_des = 6*a1.*t + 2*b1;
t2_ddot_des = 6*a2.*t + 2*b2;
plot(t,t1_ddot_des,'lineWidth',2.0);
hold on
plot(t,t2_ddot_des,'lineWidth',2.0);
xlabel('Time(sec)');
ylabel('ThetaDdot(rad/s^2)');
legend('Theta1Ddot','Theta2Ddot');
title('ThetaDdot v/s. Time');

syms M1 M2 M3 M4 C1 C2 C3 C4 g1 g2 real
syms l1 l2 r1 r2 t1 t2 t1_dot t2_dot m1 m2 I1 I2 g u1 u2 t1_ddot t2_ddot real %X1 X2 X3 X4 %t stands for theta
syms a b d a_hat b_hat d_hat real
syms m1_hat m2_hat I1_hat I2_hat real

q = [t1;t2];
q_dot = [t1_dot;t2_dot];
q_ddot = [t1_ddot; t2_ddot];
q_des = [t1_des;t2_des];
q_dot_des = [t1_dot_des;t2_dot_des];
q_ddot_des = [t1_ddot_des;t2_ddot_des];
e = q - q_des;
e_dot = q_dot - q_dot_des;
e_ddot = q_ddot - q_ddot_des;
x = [e;e_dot];
x_dot = [e_dot;e_ddot];
% x_des = [q_des;q_dot_des];

m1_hat = 0.75;
m2_hat = 0.75;
I1_hat = 0.063;
I2_hat = 0.063;
m1=1;
m2=1;
l1=1;
l2=1;
r1=0.45;
r2=0.45;
I1=0.084;
I2=0.084;
g=9.81;

M = [a+2*b*cos(t2), d+b*cos(t2); d+b*cos(t2), d];
C = [-b*sin(t2)*t2_dot, -b*sin(t2)*(t1_dot+t2_dot); b*sin(t2)*t1_dot,0];
G = [-m1*g*r1*sin(t1)-m2*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2*g*r2*sin(t1+t2)];
% where
a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
b = m2*l1*r2;
d = I2 + m2*r2^2;
% T = M*q_ddot + C*q_dot + G;

M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
% where
a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
b_hat = m2_hat*l1*r2;
d_hat = I2_hat + m2_hat*r2^2;
% T = M_hat*q_ddot + C_hat*q_dot + G_hat;