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
m1_hat=0.75;m2_hat=0.75;l1=1;l2=1;r1=0.45;r2=0.45;I1_hat=0.063;I2_hat=0.063;g=9.81;

t = 0;
Kn = [12	0	7	0;
        0	12	0	7];
kp = Kn(:,1:2);
kd = Kn(:,3:4);

Acl = [0,0,1,0;0,0,0,1;-kp(1,:),-kd(1,:);-kp(2,:),-kd(2,:)];
Q = eye(4).*2;
P = lyap(Acl',Q);

B = [0,0;0,0;1,0;0,1];
rho = eye(2).*8;
phi = 0.1;
i = 1;

while(t < 10)
    
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure

% design your state feedback controller in the following

q_des = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;(pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2];
q_dot_des = [(3*pi*t^2)/500 - (3*pi*t)/50;(3*pi*t^2)/1000 - (3*pi*t)/100];
q_ddot_des = [(3*pi*t)/250 - (3*pi/50);(3*pi*t/500)-(3*pi*t/100)];

x = [jointData.Position;jointData.Velocity];
x_des = [q_des;q_dot_des];
e = x(1:2) - x_des(1:2);
e_dot = x(3:4) - x_des(3:4);
E = [e;e_dot];
t1 = x(1);
t2 = x(2);
t1_dot = x(3);
t2_dot = x(4);

a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
b_hat = m2_hat*l1*r2;
d_hat = I2_hat + m2_hat*r2^2;
M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
% M = [(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2);
%     (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2), (m2*r2^2 + I2)];
% C = [- (m2*t2_dot*(2*l1*r2*sin(t2)+2*l1*r2*sin(t2)))/2, - (m2*t2_dot*(2*l1*r2*sin(t2)));
%     l1*m2*r2*t1_dot*sin(t2) - l1*m2*r2*t2_dot*sin(t2), l1*m2*r2*t1_dot*sin(t2)];
% G = [- g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1));
%     - g*m2*r2*sin(t1 + t2)];

% vd = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];


% v = -Kn*(x - x_des) + vd;
% u = M*v + C*q_dot + G;

if norm(E'*P*B)>phi
    vr = -((E'*P*B)/(norm(E'*P*B)))*rho;
else
    vr = -((E'*P*B)*rho/phi);
end

v = q_ddot_des - kp*e - kd*e_dot + vr';
u = M_hat*v + C_hat*[x(3);x(4)] + G_hat;

tau1.Data = u(1);
tau2.Data = u(2);

send(j1_effort,tau1);
send(j2_effort,tau2);
% you can sample data here to plot at the end

xi(:,i) = x;
ui(:,i) = u;
ti(:,i) = t;
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