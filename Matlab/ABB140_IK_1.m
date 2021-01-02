clear all
clc
close all
%%
syms q1 q2 q3 q4 q5 q6

L0=0.07;
L1=0.3520;
L2=0.360;
L3=0.380;
L4=0.065;
Le = 0.1574;% distance from J6 to Tool in Coppelia

T00 = eye(4);
T10 = (DH(q1,L1,L0,sym(pi)/2)); 
T21 = (DH(q2,0,L2,0));  
T32 = (DH(q3,0,0,sym(pi)/2));   
T43 = (DH(q4,L3,0,-sym(pi)/2));
T54 = (DH(q5,0,0,sym(pi)/2));
T65 = (DH(q6,L4,0,0));
TE6 = DH(0,Le,0,0);

T20 = (T10*T21);
T30 = (T20*T32);
T40 = (T30*T43);
T50 = (T40*T54);
T60 = (T50*T65);
TE0 = T60*TE6;
%% Robotics toolbox
t = [0 0 0 0 0 0];
L(1) = Link([t(1),L1,L0,(pi)/2],'standard'); %theta,d,a,alpha
L(2) = Link([t(2),0,L2,0],'standard');
L(3) = Link([t(3),0,0,pi/2],'standard');
L(4) = Link([t(4),L3,0,-(pi)/2],'standard');
L(5) = Link([t(5),0,0,(pi)/2],'standard');
L(6) = Link([t(6),L4,0,0],'standard');
tip = [0 0 Le]'; % end effector position
R = eye(3);
tool = [R tip;0 0 0 1]; 
IRB140 = SerialLink(L,'name','IRB140','tool',tool);
% figure(1)
% IRB140.teach(double(t))
% hold on
% plot3(Tik(1,4),Tik(2,4),Tik(3,4),'ro')
% plot3(value(1,4),value(2,4),value(3,4),'b*')
% legend('Target','IK Solution')
%% Traiettoria circolare
r = 0.5;
j = 1;

for i = 0:0.1:2*pi
    x(j) = r*cos(i);
    y(j) = r*sin(i);
    z(j) = 0.5;
    j = j+1;   
end
%% IK
h = waitbar(0,'Please wait...');
for i = 1:length(x)
p = [x(i) y(i) z(i)]';
R = eul2r([0 -pi/4 0]);
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
set_1(i,:) = s1;
set_2(i,:) = s2;
set_3(i,:) = s3;
set_4(i,:) = s4;
set_5(i,:) = s5;
set_6(i,:) = s6;
set_7(i,:) = s7;
set_8(i,:) = s8;
waitbar(i/length(x),h)
end
%% Check for feasible solution
set1 = checkLimits(set_1);
set2 = checkLimits(set_2);
set3 = checkLimits(set_3);
set4 = checkLimits(set_4);
set5 = checkLimits(set_5);
set6 = checkLimits(set_6);
set7 = checkLimits(set_7);
set8 = checkLimits(set_8);
%% 
close all
figure(2)
hold on,grid on
view(3)
plot3(x,y,z,'r');
xlim([-1 1]);ylim([-1 1]);zlim([0 1.5]);
for i = 1:length(set1)
IRB140.plot(set1(i,:));
end

save Cerchio
%% Plot 
min = deg2rad([-180 -90 -230 -200 -115 -400]);
max = deg2rad([180 110 50 200 115 400]);

set = set1;
figure(2)
subplot(321)
hold on,grid on
title('\theta_{1}')
plot(set(:,1));xlabel('time [s]');
yline(min(1),'r'),yline(max(1),'r')
subplot(322)
hold on,grid on
title('\theta_{2}')
plot(set(:,2));xlabel('time [s]');

subplot(323)
hold on,grid on
title('\theta_{3}')
plot(set(:,3));xlabel('time [s]');
subplot(324)
hold on,grid on
title('\theta_{4}')
plot(set(:,4));xlabel('time [s]');
subplot(325)
hold on,grid on
title('\theta_{5}')
plot(set(:,5));xlabel('time [s]');
subplot(326)
hold on,grid on
title('\theta_{6}')
plot(set(:,6));xlabel('time [s]');







