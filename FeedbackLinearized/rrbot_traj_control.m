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
t = 0;
Kn = [26.3636   -0.9759   10.2714   -0.0273;
    0.8324   20.6796    1.1149    9.7286];
m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
xi = [];
ui = [];
ti = [];
n = 1;
while(t < 10)
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure

% design your state feedback controller in the following
x = [jointData.Position;jointData.Velocity];
t1 = x(1);
t2 = x(2);
t1_dot = x(3);
t2_dot = x(4);

q_des = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;(pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2];
q_dot_des = [(3*pi*t^2)/500 - (3*pi*t)/50;(3*pi*t^2)/1000 - (3*pi*t)/100];

M = [(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2);
    (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2), (m2*r2^2 + I2)];
C = [- (m2*t2_dot*(2*l1*r2*sin(t2)+2*l1*r2*sin(t2)))/2, - (m2*t2_dot*(2*l1*r2*sin(t2)));
    l1*m2*r2*t1_dot*sin(t2) - l1*m2*r2*t2_dot*sin(t2), l1*m2*r2*t1_dot*sin(t2)];
G = [- g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1));
    - g*m2*r2*sin(t1 + t2)];

vd = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];

x_des = [q_des;q_dot_des];

v = -Kn*(x - x_des) + vd;

u = M*v + C*jointData.Velocity + G;

tau1.Data = u(1);
tau2.Data = u(2);

send(j1_effort,tau1);
send(j2_effort,tau2);
% you can sample data here to plot at the end
xi(:,n) = [jointData.Position;jointData.Velocity];
ui(:,n) = [jointData.Effort];
ti(:,n) = t;
    
n = n+1;
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
plot(ti,xi(1,:));
hold on;
plot(ti,t1_des);
xlabel('time (sec)');
ylabel('t1 (radian)');

figure;
plot(ti,xi(2,:));
hold on;
plot(ti,t1_des);
xlabel('time (sec)');
ylabel('t2 (radian)');

figure;
plot(ti,xi(3,:));
hold on;
plot(ti,t1_dot_des);
xlabel('time (sec)');
ylabel('theta1 (radian/sec)');

figure;
plot(ti,xi(4,:));
hold on;
plot(ti,t2_dot_des);
xlabel('time (sec)');
ylabel('theta2 (radian/sec)');

figure;
plot(ti,ui(1,:));
xlabel('time (sec)');
ylabel('u1 (units)');

figure
plot(ti,ui(2,:));
xlabel('time (sec)');
ylabel('u2 (units)');