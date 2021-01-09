%% Main file 
clear all
close all
clc

run createRobot
Ts = 0.005;

% 2 displayed frames result in 1 recorded frame,100fps
% bullet 2.78, very accurate,dt = 200ms

% LOAD CHECKPOINTFINALE.MAT
%% Traiettoria 1 -- spazio dei giunti 
% Condizioni iniziali
qi1 = [0 +pi/2 0 0 0 -pi/2];
dqi1 = zeros(1,6);
ddqi1 = zeros(1,6);
ti1 = 0;

% Condizioni finali 
qf1 = [-1.8425    1.2918   -0.1196    1.4631    1.3208   -2.7295];
dqf1 = zeros(1,6);
ddqf1 = zeros(1,6);
tf1 = 1;

[q1_1,q1_2,q1_3,q1_4,q1_5,q1_6] = trajPlan1(qi1,qf1,ti1,tf1,dqi1,dqf1,ddqi1,ddqf1,Ts);

% Plate
q1_7 = zeros(1,length(q1_1));
q1_8 = zeros(1,length(q1_1));
res1 = [q1_1', q1_2', q1_3', q1_4', q1_5', q1_6'];
res1_p = [q1_7', q1_8'];
% load ('checkpoint1')
% save('checkpoint1')
%% Traiettoria 2 -- spazio operativo/segmento lineare dall'alto al basso
Pin2 = [-0.3454  -0.5003   0.5506]';
Pf2 = [-0.3454  -0.5003   0.3306]';
ti2 = tf1;
tf2 = tf1 + 1;
[p_s2,dp_s2,ddp_s2] = trajPlan2(Pin2,Pf2,ti2,tf2,Ts);
tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res1(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s2)
    R_t2(i).R = tmp(1:3,1:3);   
end
res2 = solveIK(p_s2,R_t2,T30);


% Plate
q2_7 = zeros(length(p_s2),1);
q2_8 = zeros(length(p_s2),1);
res2_p = [q2_7 q2_8];
% load('checkpoint2')
% save('checkpoint2')
%% Traiettoria 3 -- spazio operativo/segmento lineare dal basso all'alto
Pin3 = [-0.3454  -0.5003   0.3306]';
Pf3 =  [-0.3454  -0.5003   0.5506]';
ti3 = tf2;
tf3 = ti3 + 1;
[p_s3,dp_s3,ddp_s3] = trajPlan2(Pin3,Pf3,ti3,tf3,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res2(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s3)
    R_t3(i).R = tmp(1:3,1:3);   
end
res3 = solveIK(p_s3,R_t3,T30);

q3_7 = linspace(0,-pi/2,length(res3));
q3_8 = zeros(1,length(res3));
res3_p = [q3_7' q3_8'];
% load('checkpoint3')
% save('checkpoint3')
%% Traiettoria 7 - da sopra il tavolo al cavatappi, spazio operativo
Pcv = [-0.2794 -0.5013 0.2706]';

Pin7_1 = Pf3;
Pf7_1 =  Pcv + [0.2 0 -0.1155]';
ti7_1 = tf3;
tf7_1 = ti7_1 + 1;
[p_s7_1,dp_s7_1,ddp_s7_1] = trajPlan2(Pin7_1,Pf7_1,ti7_1,tf7_1,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res3(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s7_1)
    R_t7_1(i).R = tmp(1:3,1:3);   
end
res7_1 = solveIK(p_s7_1,R_t7_1,T30);

Pin7_2 = Pf7_1;
Pf7_2 =  Pcv + [0 0 -0.1155]';
ti7_2 = tf7_1;
tf7_2 = ti7_2 + 1;
[p_s7_2,dp_s7_2,ddp_s7_2] = trajPlan2(Pin7_2,Pf7_2,ti7_2,tf7_2,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res3(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s7_2)
    R_t7_2(i).R = tmp(1:3,1:3);   
end
res7_2 = solveIK(p_s7_2,R_t7_2,T30);
res7 = [res7_1;res7_2];

q7_7 = -pi/2*ones(1,length(res7));
q7_8 = zeros(1,length(res7));
res7_p = [q7_7' q7_8'];
% save('checkpoint7')
%% Traiettoria 4 - per passare sopra il tavolo--spazio dei giunti
% Condizione iniziale

ti4 = tf3;
tf4 = tf3 + 1;
qi4 = [-1.8425    1.2918   -0.1196    1.4631    1.3208   -2.7295];
qf4 = [-1.5708    1.2275   -0.0351   -0.0002    0.3783   -1.5706];
dqi4 = zeros(1,6);
ddqi4 = zeros(1,6);
dqf4 = zeros(1,6);
ddqf4 = zeros(1,6);
[q4_1,q4_2,q4_3,q4_4,q4_5,q4_6] = trajPlan1(qi4,qf4,ti4,tf4,dqi4,dqf4,ddqi4,ddqf4,Ts);
res4 = [q4_1',q4_2',q4_3',q4_4',q4_5',q4_6'];
for i = 1:length(res4)
tmp = double(IRB140.fkine(res4(i,:)));
p_s4(:,i) = tmp(1:3,end);
end
q4_7 = -pi/2*ones(1,length(res4));
q4_8 = zeros(1,length(res4));
res4_p = [q4_7' q4_8'];
% load('checkpoint4')
% save('checkpoint4')
%% Traiettoria 5 -- spazio operativo/circolare
pt = p_s4(:,end);                   % Punto appartenente alla crf
r = [0 0 1]';                       % versore asse perpendicolare al piano della crf
d = [pt(1)+0.02 pt(2) pt(3)+1]';    %         
delta = pt-d;
c = d + (delta'*r)*r;
ro = norm(pt-c);
s = trajPlan2(0,2*pi*ro,0,2,Ts);
cfr = trajCircle(r,d,pt,s);

tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res4(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(cfr)
    R_t5(i).R = tmp(1:3,1:3);   
end
res5 = solveIK(cfr,R_t5,T30);
q5_7 = -pi/2*ones(1,length(res5));
q5_8 = zeros(1,length(res5));
res5_p = [q5_7' q5_8'];
% load checkpoint5
% save('checkpoint5')
%% Traiettoria 6 -- Bottle Shake
% Calcolo matrice di rotazione per miscelare la birra
T = shakeBeer(cfr,c,ro);

% Risoluzione cinematica inversa
res6 = solveIK(cfr,T,T30);

q6_7 = -pi/2*ones(1,length(res6));
q6_8 = zeros(1,length(res6));
res6_p = [q6_7' q6_8'];
% load checkpoint6
% save('checkpoint6')
%% Accorpo traiettoria 1,2,3,(7),4,5,6 (giunti + lineare + lineare + (lineare) + lineare + cfr + shake)
res = [res1;res2;res3;res4;res5;res6];
% res = [res1;res2;res3;res7]; % prende la birra e la stappa
res_p = [res1_p;res2_p;res3_p;res4_p;res5_p;res6_p];% prende la birra e shakera (no stappo)
% load('checkpointFinale')
%% Plot Robotic Toolbox

for i = 1:length(res)
tmp = double(IRB140.fkine(res(i,:)));
ee(:,i) = tmp(1:3,end);
end
close all

figure(2)
hold on,grid on
view(3)
plot3(ee(1,:),ee(2,:),ee(3,:),'r');
xlim([-1 1]);ylim([-1 1]);zlim([0 1.5]);
for i = 1:10:length(res)
IRB140.plot(res(i,:));
% pause(0.5)
end
% IRB140.plot(qf);
