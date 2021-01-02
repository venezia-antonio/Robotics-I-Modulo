%% Main file 
clear all
close all
clc

% Definizione matrici di trasformazione
syms q1 q2 q3 q4 q5 q6

global L0 L1 L2 L3 L4 Le

L0=0.07;
L1=0.3520;
L2=0.360;
L3=0.380;
L4=0.046;
Le = 0.16;
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

% Definizione robot con robotic toolbox
theta = [0 0 0 0 0 0];
L(1) = Link([theta(1),L1,L0,(pi)/2],'standard'); %theta,d,a,alpha
L(2) = Link([theta(2),0,L2,0],'standard');
L(3) = Link([theta(3),0,0,pi/2],'standard');
L(4) = Link([theta(4),L3,0,-(pi)/2],'standard');
L(5) = Link([theta(5),0,0,(pi)/2],'standard');
L(6) = Link([theta(6),L4,0,0],'standard');
tip = [0 0 Le]'; % end effector position
R = eye(3);
tool = [R tip;0 0 0 1]; 
IRB140 = SerialLink(L,'name','IRB140','tool',tool);

%% Traiettoria 1 -- spazio dei giunti 
% Condizioni iniziali
qi = [0 +pi/2 0 0 0 -pi/2];
dqi = zeros(1,6);
ddqi = zeros(1,6);
ti1 = 0;
% Condizioni finali 
qf = [pi/4, 0+pi/2, -pi/4, 0, pi/4, -pi/2];
dqf = zeros(1,6);
ddqf = zeros(1,6);
tf1 = 5;

[pos1,pos2,pos3,pos4,pos5,pos6] = trajPlan1(qi,qf,ti1,tf1,dqi,dqf,ddqi,ddqf);
pos = [pos1', pos2', pos3', pos4', pos5', pos6'];
%% Seconda traiettoria 
Pin = [0.3851, 0.3852, 0.4434]';
% Pin = [0.3064 0.3067 0.4433]';
% Pf = [0.3064 0.3067 0.2433]';
Pf = [0.3851, 0.3852, 0.2434]';
ti2 = 5;
tf2 = 7;


[p_s,dp_s,ddp_s] = trajPlan2(Pin,Pf,ti2,tf2);

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

end

%% Plot traiettoria complessiva 
result = [pos;set2];
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
figure()
subplot(311)
plot(0:0.01:5,dp_s(1,:))
subplot(312)
plot(0:0.01:5,dp_s(2,:))
subplot(313)
plot(0:0.01:5,dp_s(3,:))

figure()
subplot(311)
plot(0:0.01:5,ddp_s(1,:))
subplot(312)
plot(0:0.01:5,ddp_s(2,:))
subplot(313)
plot(0:0.01:5,ddp_s(3,:))