%% Main file 
clear all
close all
clc

run createRobot

%% Traiettoria 1 -- spazio dei giunti 
% Condizioni iniziali
qi = [0 +pi/2 0 0 0 -pi/2];
dqi = zeros(1,6);
ddqi = zeros(1,6);
ti1 = 0;
% Condizioni finali 
qf = [-1.8425    1.2918   -0.1196    1.4631    1.3208   -2.7295];
dqf = zeros(1,6);
ddqf = zeros(1,6);
tf1 = 1;

[pos1,pos2,pos3,pos4,pos5,pos6] = trajPlan1(qi,qf,ti1,tf1,dqi,dqf,ddqi,ddqf);
pos = [pos1', pos2', pos3', pos4', pos5', pos6'];
%% Traiettoria 2 -- spazio operativo/lineare
Pin = [-0.3454  -0.5003   0.5506]';
Pf = [-0.3454  -0.5003   0.3306]';
ti2 = tf1;
tf2 = tf1 + 1;
[p_s1,dp_s1,ddp_s1] = trajPlan2(Pin,Pf,ti2,tf2);

%% Traiettoria 3 -- spazio operativo/lineare
Pin = [-0.3454  -0.5003   0.3306]';
Pf =  [-0.3454  -0.5003   0.5506]';
ti3 = tf2;
tf3 = tf2 + 1;
[p_s2,dp_s2,ddp_s2] = trajPlan2(Pin,Pf,ti3,tf3);
%% Traiettoria 4 -- spazio operativo/circolare
pt = p_s2(:,end);
r = [0 0 1]';
d = [pt(1)+0.05 pt(2) pt(3)+1]';
delta = pt-d;
c = d + (delta'*r)*r;
ro = norm(pt-c);
s = trajPlan2(0,2*pi*ro,0,2);
cfr = trajCircle(r,d,pt,s);
%% Accorpo traiettoria 2,3,4 (lineare + lineare + cfr)
p_s(1,:) = [p_s1(1,:) p_s2(1,:) cfr(1,:)];
p_s(2,:) = [p_s1(2,:) p_s2(2,:) cfr(2,:)];
p_s(3,:) = [p_s1(3,:) p_s2(3,:) cfr(3,:)];
% save checkpoint1
%% Traiettoria 5 -- Bottle Shake



%% Inverse Kinematics
h = waitbar(0,'Please wait...');
for i = 1:length(p_s)
% Impostare posizione e orientamento desiderato
p = [p_s(1,i) p_s(2,i) p_s(3,i)]';
R = double(subs(TE0(1:3,1:3),{q1,q2,q3,q4,q5,q6},{qf}));
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
set1(i,:) = s1;
set2(i,:) = s2;
set3(i,:) = s3;
set4(i,:) = s4;
set5(i,:) = s5;
set6(i,:) = s6;
set7(i,:) = s7;
set8(i,:) = s8;
waitbar(i/length(p_s),h)
end
close(h);
%% Plot traiettoria complessiva 
result = [pos;set2]; % joint variable
for i = 1:length(result)
tmp = double(IRB140.fkine(result(i,:)));
ee(:,i) = tmp(1:3,end);
end
p_s = [p_s ee];
%%
close all
figure(2)
hold on,grid on
view(3)
plot3(p_s(1,:),p_s(2,:),p_s(3,:),'r');
xlim([-1 1]);ylim([-1 1]);zlim([0 1.5]);
for i = 1:10:length(result)
IRB140.plot(result(i,:));
end
% IRB140.plot(qf);
%%
% figure()
% subplot(311)
% plot(0:0.01:5,dp_s(1,:))
% subplot(312)
% plot(0:0.01:5,dp_s(2,:))
% subplot(313)
% plot(0:0.01:5,dp_s(3,:))
% 
% figure()
% subplot(311)
% plot(0:0.01:5,ddp_s(1,:))
% subplot(312)
% plot(0:0.01:5,ddp_s(2,:))
% subplot(313)
% plot(0:0.01:5,ddp_s(3,:))