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
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
tic;
t = 0;
u=[];
sec=[];
q=[];
i=1;
Kn = [33.8023,6.4772,14.8380,4.4190];
Knn= [8.6340,5.5361,4.4190,1.7190];
while(t < 10)
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following
tau1.Data = -Kn*[jointData.Position;jointData.Velocity];
tau2.Data = -Knn*[jointData.Position;jointData.Velocity];
send(j1_effort,tau1);
send(j2_effort,tau2);
% you can sample data here to plot at the end
q(:,i)= [jointData.Position;jointData.Velocity];
u(:,i)= jointData.Effort;
sec(:,i)= t;
i=i+1;
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore

figure;
plot(sec,rad2deg(q(1,:)));
xlabel('time (sec)');
ylabel('theta1 (degree)');
figure;
plot(sec,rad2deg(q(2,:)));
xlabel('time (sec)');
ylabel('theta2 (degree)');
figure;
plot(sec,rad2deg(q(3,:)));
xlabel('time (sec)');
ylabel('theta1_dot (degree/sec)');
figure;
plot(sec,rad2deg(q(4,:)));
xlabel('time (sec)');
ylabel('theta2_dot (degree/sec)');
figure;
plot(sec,u(1,:));
xlabel('time (sec)');
ylabel('u1 (N.m)');
figure;
plot(sec,u(2,:));
xlabel('time (sec)');
ylabel('u2 (N.m)');

rosshutdown;