clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);

tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);
tic;

% a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
% b_hat = m2_hat*l1*r2;
% d_hat = I2_hat + m2_hat*r2^2;
% M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
% C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
% G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2; m1*r1 + m2*l1; m2*r2];
alpha_hat = 0.75*alpha;

t = 0;
t_prev = 0;
t1_dot_prev = 0;
t2_dot_prev = 0;

Kn = [12	0	7	0;
        0	12	0	7];
kp = Kn(:,1:2);
kd = Kn(:,3:4);

Acl = [0,0,1,0;0,0,0,1;-kp(1,:),-kd(1,:);-kp(2,:),-kd(2,:)];
Q = eye(4);
P = lyap(Acl',Q);
Gamma = eye(5).*11;

B = [0,0;0,0;1,0;0,1];
i = 1;

while(t < 10)
    
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure

% design your state feedback controller in the following

q_des = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;(pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2];
q_dot_des = [(3*pi*t^2)/500 - (3*pi*t)/50;(3*pi*t^2)/1000 - (3*pi*t)/100];
q_ddot_des = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];

% M = [(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2);
%     (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2), (m2*r2^2 + I2)];
% C = [- (m2*t2_dot*(2*l1*r2*sin(t2)+2*l1*r2*sin(t2)))/2, - (m2*t2_dot*(2*l1*r2*sin(t2)));
%     l1*m2*r2*t1_dot*sin(t2) - l1*m2*r2*t2_dot*sin(t2), l1*m2*r2*t1_dot*sin(t2)];
% G = [- g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1));
%     - g*m2*r2*sin(t1 + t2)];
% vd = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];

x = [jointData.Position;jointData.Velocity];
q_ddot = [1000*(x(3)-t1_dot_prev)/1000*(t-t_prev);1000*(x(4)-t2_dot_prev)/1000*(t-t_prev)];
x_des = [q_des;q_dot_des];
e = x(1:2) - x_des(1:2);
e_dot = x(3:4) - x_des(3:4);
E = [e;e_dot];
t1 = x(1);
t2 = x(2);
t1_dot = x(3);
t2_dot = x(4);
% v = -Kn*(x - x_des) + vd;
% u = M*v + C*q_dot + G;

% if norm(E'*P*B)>phi
%     vr = -((E'*P*B)/(norm(E'*P*B)))*rho;
% else
%     vr = -((E'*P*B)/phi)*rho;
% end

v = q_ddot_des - kp*e - kd*e_dot;

Yo = [v(1), cos(t2)*(2*v(1) + v(2)) - 2*sin(t2)*t1_dot*t2_dot - sin(t2)*t2_dot^2, v(2), -sin(t1)*g, -sin(t1 + t2)*g;
    0, sin(t2)*t1_dot^2 + cos(t2)*v(1), v(1) + v(2), 0, -sin(t1+t2)*g];

u = Yo*alpha_hat;

t1_ddot = q_ddot(1);
t2_ddot = q_ddot(2);

Y = [t1_ddot, cos(t2)*(2*t1_ddot + t2_ddot) - 2*sin(t2)*t1_dot*t2_dot - sin(t2)*t2_dot^2, t2_ddot, -sin(t1)*g, -sin(t1 + t2)*g;
    0, sin(t2)*t1_dot^2 + cos(t2)*t1_ddot, t1_ddot + t2_ddot, 0, -sin(t1+t2)*g];

M1 = [1,2*cos(t2),0,0,0];
M2 = [0,cos(t2),1,0,0];
M3 = [0,cos(t2),1,0,0];
M4 = [0,0,1,0,0];
M_hat = [[M1;M3]*alpha_hat,[M2;M4]*alpha_hat];

B = [0,0;0,0;1,0;0,1];
Phi = M_hat\Y;
alpha_dot = -Gamma\(Phi'*B'*P*x);
alpha_hat = alpha_dot*(t-t_prev) + alpha_hat;

tau1.Data = u(1);
tau2.Data = u(2);

send(j1_effort,tau1);
send(j2_effort,tau2);
% you can sample data here to plot at the end

xi(:,i) = [jointData.Position;jointData.Velocity];
ui(:,i) = [jointData.Effort];
t_prev = t;
ti(:,i) = t;
t1_dot_prev = t1_dot;
t2_dot_prev = t2_dot;
i = i+1;
end

tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;

t1_des = (pi*ti.^3)/500 - (3*pi*ti.^2)/100 + pi;
t2_des = (pi*ti.^3)/1000 - (3*pi*ti.^2)/200 + pi/2;
t1_dot_des = (3*pi*ti.^2)/500 - (3*pi*ti)/50;
t2_dot_des = (3*pi*ti.^2)/1000 - (3*pi*ti)/100;

%Plotting the output
figure;
plot(ti,xi(1,:),'LineWidth',2);
hold on;
plot(ti,t1_des);
xlabel('time (sec)');
ylabel('t1 (radian)');

figure;
plot(ti,xi(2,:),'LineWidth',2);
hold on;
plot(ti,t2_des);
xlabel('time (sec)');
ylabel('t2 (radian)');

figure;
plot(ti,xi(3,:),'LineWidth',2);
hold on;
plot(ti,t1_dot_des);
xlabel('time (sec)');
ylabel('theta1 (radian/sec)');

figure;
plot(ti,xi(4,:),'LineWidth',2);
hold on;
plot(ti,t2_dot_des);
xlabel('time (sec)');
ylabel('theta2 (radian/sec)');

figure;
plot(ti,ui(1,:),'LineWidth',2);
xlabel('time (sec)');
ylabel('u1 (units)');

figure
plot(ti,ui(2,:),'LineWidth',2);
xlabel('time (sec)');
ylabel('u2 (units)');