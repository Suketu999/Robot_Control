syms l1 l2 r1 r2 t1 t2 t1_dot t2_dot m1 m2 I1 I2 g u1 u2 t t1_ddot t2_ddot real

q = [t1;t2];
q_dot = [t1_dot;t2_dot];
q_ddot = [t1_ddot; t2_ddot];
x = [q;q_dot];

q_des = [t1_des;t2_des];
q_dot_des = [t1_dot_des;t2_dot_des];
x_des = [q_des;q_dot_des];

q_ddot_des = [t1_ddot_des;t2_ddot_des];

eqn1 = t2_ddot*(I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2) - g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1)) - u1 + t1_ddot*(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2) - (m2*t2_dot*(2*l1*r2*sin(t2)*(t1_dot + t2_dot) + 2*l1*r2*t1_dot*sin(t2)))/2 == 0;
eqn2 = t2_ddot*(m2*r2^2 + I2) - u2 + t1_ddot*(I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2) - g*m2*r2*sin(t1 + t2) + l1*m2*r2*t1_dot*sin(t2)*(t1_dot + t2_dot) - l1*m2*r2*t1_dot*t2_dot*sin(t2) == 0;

M = [(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2);
    (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2), (m2*r2^2 + I2)];
C = [- (m2*t2_dot*(2*l1*r2*sin(t2)+2*l1*r2*sin(t2)))/2, - (m2*t2_dot*(2*l1*r2*sin(t2)));
    l1*m2*r2*t1_dot*sin(t2) - l1*m2*r2*t2_dot*sin(t2), l1*m2*r2*t1_dot*sin(t2)];
G = [- g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1));
    - g*m2*r2*sin(t1 + t2)];

T = M*q_ddot + C*q_dot + G;

% % Let virtual input v be such that:
% % T = M*v + C*q_dot + G
% % Thus, v = q_ddot
% % Let x = [q; q_dot]
% % Therefore, x_dot = [q_dot; q_ddot] = [q_dot; v] = [x3; x4; v1; v2]

A = [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B = [0,0;0,0;1,0;0,1];

lambda = [-3;-7;-5-j;-5+j];
disp('State-Feedback control is given by u = -kx. The value of k is as follows:');
Kn = place(A,B,lambda);
v = -Kn*(x - x_des) + q_ddot_des

u = M*v + C*q_dot + G;

[t,z] = ode45(@ode_progassn, [0,10], (pi/180*[200; 125; 0; 0]));

figure(1);
plot (t,(z(:,1)),'lineWidth',2.0);
xlabel('t(sec)') 
ylabel('Theta 1') 
hold on
plot (t,(z(:,2)),'lineWidth',2.0);
title('Theta v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 2') 
legend('Theta1','Theta2')

figure(2);
plot (t,(z(:,3)),'lineWidth',2.0);
xlabel('t(sec)') 
ylabel('Theta 1 dot')
hold on
plot (t,(z(:,4)),'lineWidth',2.0);
title('ThetaDot v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 2 dot')
legend('Theta1dot','Theta2dot')

f=1;
U_1 = 0;
U_2 = 0;
zsize =size(z);
while f <= zsize(1)
    U_1(f,1) = - (1189312871105339*z(f,1))/35184372088832 - (7292630729812249*z(f,2))/1125899906842624 - (7419*z(f,3))/500 - (4419*z(f,4))/1000;
    U_2(f,1) = - (2430265662578031*z(f,1))/281474976710656 - (6233124955178157*z(f,2))/1125899906842624 - (4419*z(f,3))/1000 - (1719*z(f,4))/1000;
    f = f+1;
end

figure(5);
plot (t,U_1,'lineWidth',2.0);
xlabel('t(sec)') 
ylabel('U1 (units)')
hold on
plot (t,U_2,'lineWidth',2.0);
title('U v/s. Time')
xlabel('t(sec)') 
ylabel('U2 (units)')
legend('U1','U2')