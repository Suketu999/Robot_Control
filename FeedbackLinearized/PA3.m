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

%t = 0:0.01:10;
syms t;
t1_des = a1*t.^3 + b1*t.^2 + c1.*t + d1
t2_des = a2*t.^3 + b2*t.^2 + c2.*t + d2

t1_dot_des = 3*a1*t.^2 + 2*b1.*t + c1
t2_dot_des = 3*a2*t.^2 + 2*b2.*t + c2

t1_ddot_des = 6*a1.*t + 2*b1
t2_ddot_des = 6*a2.*t + 2*b2

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