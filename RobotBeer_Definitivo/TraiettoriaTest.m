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
tf1 = 2;

[q1_1,q1_2,q1_3,q1_4,q1_5,q1_6] = trajPlan1(qi1,qf1,ti1,tf1,dqi1,dqf1,ddqi1,ddqf1,Ts);

% Plate
q1_7 = zeros(1,length(q1_1));
q1_8 = zeros(1,length(q1_1));
res1 = [q1_1', q1_2', q1_3', q1_4', q1_5', q1_6'];
res1_p = [q1_7', q1_8'];
% load ('checkpoint1')
save('checkpoint1')
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
save('checkpoint2')
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
save('checkpoint3')
%% Traiettoria 4 - da sopra il tavolo al cavatappi, spazio operativo
Pcv = [-0.2794 -0.5013 0.2706]';

Pin4_1 = Pf3;
Pf4_1 =  Pcv + [0.2 0 -0.1155]';
ti4_1 = tf3;
tf4_1 = ti4_1 + 1;
[p_s4_1,dp_s4_1,ddp_s4_1] = trajPlan2(Pin4_1,Pf4_1,ti4_1,tf4_1,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res3(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s4_1)
    R_t4_1(i).R = tmp(1:3,1:3);   
end
res4_1 = solveIK(p_s4_1,R_t4_1,T30);

Pin4_2 = Pf4_1;
Pf4_2 =  Pcv + [0 0 -0.1155]';
ti4_2 = tf4_1;
tf4_2 = ti4_2 + 1;
[p_s4_2,dp_s4_2,ddp_s4_2] = trajPlan2(Pin4_2,Pf4_2,ti4_2,tf4_2,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res3(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s4_2)
    R_t4_2(i).R = tmp(1:3,1:3);   
end
res4_2 = solveIK(p_s4_2,R_t4_2,T30);
res4 = [res4_1;res4_2];

% Angoli tavolo
q4_7 = -pi/2*ones(1,length(res4));
q4_8 = zeros(1,length(res4));
res4_p = [q4_7' q4_8'];
save('checkpoint4')
%% Traiettoria 5 - Stappo
ti5 = tf4_2;
tf5 = ti5 + 5;
radius = 0.1155;
R5 = rotz(-pi/2)*roty(pi/2) ;
pt1 = p_s4_2(:,end);                   % Punto appartenente alla crf
r1 = R5*[0 0 1]';                       % versore asse perpendicolare al piano della crf


d1  = R5*[-radius 0 -1]' + pt1;
delta1 = pt1-d1;
c1 = d1 + (delta1'*r1)*r1;
ro1 = norm(pt1-c1);
s1 = trajPlan2(0,pi*ro1/4,ti5,tf5,Ts);
cfr1 = trajCircle(r1,d1,pt1,s1,R5);

figure(1)
    grid on,hold on
    axis equal
    view(3)
    plot3(cfr1(1,:),cfr1(2,:),cfr1(3,:),'b')
    hold on
    plot3(c1(1),c1(2),c1(3),'ro')
    plot3(d1(1),d1(2),d1(3),'r*')
    plot3(pt1(1),pt1(2),pt1(3),'bx')
xlabel('X');ylabel('Y');zlabel('Z')
alpha5 = 0;
R5_1 = roty(pi/2);
T5 = shakeBeer(cfr1,c1,ro1,alpha5,R5_1);
res5 = solveIK(cfr1,T5,T30);

q5_7 = -pi/2*ones(1,length(res5));
q5_8 = zeros(1,length(res5));  
res5_p = [q5_7' q5_8'];
save checkpoint5
%% Traiettoria 6 - Mi allontano dal cavatappi / spazio operativo
Pin6 = cfr1(:,end);
Pf6 =  Pin6 + [0.06 0 0]';
ti6 = tf5;
tf6 = ti6 + 2;
[p_s6,dp_s6,ddp_s6] = trajPlan2(Pin6,Pf6,ti6,tf6,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res5(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s6)
    R_t6(i).R = tmp(1:3,1:3);   
end
res6 = solveIK(p_s6,R_t6,T30);

% Plate
q6_7 = -pi/2*ones(1,length(res6));
q6_8 = zeros(1,length(res6));         % da runnare
res6_p = [q6_7' q6_8'];
save checkpoint6
%% Traiettoria 7 - Approccio al bicchiere / spazio dei giunti
% Condizioni iniziali
qi7 = res6(end,:);
dqi7 = zeros(1,6);
ddqi7 = zeros(1,6);
ti7 = tf6;

% Condizioni finali 
qf7 = [-2.1971    1.1262   -0.7534   -0.6605    1.2712    0.2254];
dqf7 = zeros(1,6);
ddqf7 = zeros(1,6);
tf7 = ti7 + 5;

[q7_1,q7_2,q7_3,q7_4,q7_5,q7_6] = trajPlan1(qi7,qf7,ti7,tf7,dqi7,dqf7,ddqi7,ddqf7,Ts);
res7 = [q7_1', q7_2', q7_3', q7_4', q7_5', q7_6'];

% Plate
q7_7 = -pi/2*ones(1,length(res7));
q7_8 = linspace(0,pi/4,length(res7));
res7_p = [q7_7' q7_8'];
save chekpoint7

%% Traiettoria 8 - Versamento 1

Pin8 = double(subs(TE0(1:3,end),{q1,q2,q3,q4,q5,q6},{res7(end,:)}));
Pf8 =  [-0.4   -0.5    0.4772]';
ti8 = tf7;
tf8 = ti8 + 5;
[p_s8,dp_s8,ddp_s8] = trajPlan2(Pin8,Pf8,ti8,tf8,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res7(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s8)
    R_t8(i).R = tmp(1:3,1:3);   
end
res8 = solveIK(p_s8,R_t8,T30);

% Plate
q8_7 = -pi/2*ones(1,length(res8));
q8_8 = linspace(pi/4,0,length(res8));         
res8_p = [q8_7(1:500)' q8_8(1:500)'];
res8 = res8(1:500,:);
save checkpoint8
%% Traiettoria 9 - Approccio Shake
index = 500;%109-30;
ti9 = tf8;
tf9 = ti9 + 2;
qi9 = res8(index,:);
qf9 = [-1.5708    1.2275   -0.0351   -0.0002    0.3783   -1.5706];
dqi9 = zeros(1,6);
ddqi9 = zeros(1,6);
dqf9 = zeros(1,6);
ddqf9 = zeros(1,6);
[q9_1,q9_2,q9_3,q9_4,q9_5,q9_6] = trajPlan1(qi9,qf9,ti9,tf9,dqi9,dqf9,ddqi9,ddqf9,Ts);
res9 = [q9_1',q9_2',q9_3',q9_4',q9_5',q9_6'];
for i = 1:length(res9)
tmp = double(IRB140.fkine(res9(i,:)));
p_s9(:,i) = tmp(1:3,end);
end
q9_7 = q8_7(index)*ones(1,length(res9));
q9_8 = q8_8(index)*ones(1,length(res9));
res9_p = [q9_7' q9_8'];
% load('checkpoint4')
save('checkpoint9')
%%  Traiettoria 10 - spazio operativo/circolare
n = 4; % numero di shake
ti10 = tf9;
tf10 = ti10 + n;
pt = p_s9(:,end);                  % Punto appartenente alla crf
r = [0 0 1]';                       % versore asse perpendicolare al piano della crf
R10 = rotz(-pi); 
d = [pt(1)+0.02 pt(2) pt(3)+1]';    %         
delta = pt-d;
c = d + (delta'*r)*r;
ro = norm(pt-c);
s = trajPlan2(0,n*2*pi*ro,ti10,tf10,Ts);
cfr = trajCircle(r,d,pt,s,R10);

tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res9(end,:)}));

%Inizializzo struttura di matrici di rotazione
for i = 1:length(cfr)
    R_t10(i).R = tmp(1:3,1:3);   
end
res10 = solveIK(cfr,R_t10,T30);
q10_7 = q8_7(index)*ones(1,length(res10));
q10_8 = q8_8(index)*ones(1,length(res10));
res10_p = [q10_7' q10_8'];
% load checkpoint5
save('checkpoint10')
% Traiettoria 11 - Shake
ti11 = tf10;
tf11 = ti11 + n;
alpha11 = sym(5*pi/12);
R11 = roty(pi);
T = shakeBeer(cfr,c,ro,alpha11,R11);
res11 = solveIK(cfr,T,T30);
q11_7 = q8_7(index)*ones(1,length(res11));
q11_8 = q8_8(index)*ones(1,length(res11));
res11_p = [q11_7' q11_8'];
save('checkpoint11')
%% Traiettoria Intermedia - Orientamento tra 10 e 11
% Condizioni iniziali
qi10_1 = res10(end,:);
dqi10_1 = zeros(1,6);
ddqi10_1 = zeros(1,6);
ti10_1 = tf10;

% Condizioni finali 
qf10_1 = res11(1,:);
dqf10_1 = zeros(1,6);
ddqf10_1 = zeros(1,6);
tf10_1 = ti10_1 + 1;

[q10_1_1,q10_1_2,q10_1_3,q10_1_4,q10_1_5,q10_1_6] = trajPlan1(qi10_1,qf10_1,ti10_1,tf10_1,dqi10_1,dqf10_1,ddqi10_1,ddqf10_1,Ts);
res10_1 = [q10_1_1', q10_1_2', q10_1_3', q10_1_4', q10_1_5', q10_1_6'];

% Plate
q10_1_7 = q8_7(index)*ones(1,length(res10_1));
q10_1_8 = q8_8(index)*ones(1,length(res10_1));
res10_1_p = [q10_1_7' q10_1_8'];
save chekpoint10_1
%% Traiettori 12 - Ritorno al bicchiere
ti12 = tf11;
tf12 = ti12 + 1;
res12 = flip(res9);
q12_7 = q8_7(index)*ones(1,length(res12));
q12_8 = q8_8(index)*ones(1,length(res12));
res12_p = [q12_7' q12_8'];
save('checkpoint12')
% Traiettoria Intermedia - Orientamento tra 11 e 12
% Condizioni iniziali
qi11_1 = res11(end,:);
dqi11_1 = zeros(1,6);
ddqi11_1 = zeros(1,6);
ti11_1 = tf11;

% Condizioni finali 
qf11_1 = res12(1,:);
dqf11_1 = zeros(1,6);
ddqf11_1 = zeros(1,6);
tf11_1 = ti11_1 + 1;

[q11_1_1,q11_1_2,q11_1_3,q11_1_4,q11_1_5,q11_1_6] = trajPlan1(qi11_1,qf11_1,ti11_1,tf11_1,dqi11_1,dqf11_1,ddqi11_1,ddqf11_1,Ts);
res11_1 = [q11_1_1', q11_1_2', q11_1_3', q11_1_4', q11_1_5', q11_1_6'];

% Plate
q11_1_7 = q8_7(index)*ones(1,length(res11_1));
q11_1_8 = q8_8(index)*ones(1,length(res11_1));
res11_1_p = [q11_1_7' q11_1_8'];
save chekpoint11_1
%% Traiettoria 13 - continuo a versare inclinando la bottiglia contemporaneament
Pin13 = double(subs(TE0(1:3,end),{q1,q2,q3,q4,q5,q6},{res8(index,:)}));
Pf13 = Pf8;
ti13 = tf12;
tf13 = ti13 + 1;
[p_s13,dp_s13,ddp_s13] = trajPlan2(Pin13,Pf13,ti13,tf13,Ts);


Rin = double(subs(TE0(1:3,1:3),{q1,q2,q3,q4,q5,q6},{res12(end,:)}));
axis = [0 1 0]';

% Genero l'angolo
% Condizioni iniziali
anglei = [0 0 0 0 0 0];
danglei = [0 0 0 0 0 0];
ddanglei = [0 0 0 0 0 0];
ti_angle = tf12;

% Condizioni finali 
anglef = [-pi/6 1 1 1 1 1];
danglef = [0 0 0 0 0 0];
ddanglef = [0 0 0 0 0 0];
tf_angle = ti_angle + 1;

[angle,~,~,~,~,~] = trajPlan1(anglei,anglef,ti_angle,tf_angle,danglei,danglef,ddanglei,ddanglef,Ts);

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s13)
       R_t13(i).R = axisAngle(axis,angle(i))*Rin;   
end
res13 = solveIK(p_s13,R_t13,T30);
% Plate
% q13_7 = q8_7(index:end);
% q13_8 = q8_8(index:end);   
q13_7 = -pi/2*ones(1,length(res13));
q13_8 = linspace(q8_8(index),0,length(res13));     
res13_p = [q13_7' q13_8'];
save checkpoint13
%% Traiettoria 14 - sgocciolo
Pin14 = p_s13(:,end);
Pf14 =  Pin14 + [0 0 0.35]';
ti14 = tf13;
tf14 = ti14 + 1;
[p_s14,dp_s14,ddp_s14] = trajPlan2(Pin14,Pf14,ti14,tf14,Ts);

tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res13(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s14)
    R_t14(i).R = tmp(1:3,1:3);   
end
res14 = solveIK(p_s14,R_t14,T30);
res14 = [res14(1:round(0.5*length(res14)),:);flip(res14(1:round(0.5*length(res14)),:));res14(1:round(0.5*length(res14)),:);flip(res14(1:round(0.5*length(res14)),:));res14];
q14_7 = -pi/2*ones(1,length(res14));
q14_8 = zeros(1,length(res14));
res14_p = [q14_7' q14_8'];
save('checkpoint14')
%% Traiettoria 15 - Vaffancuuuloooooooo
ti15 = tf14;
tf15 = ti15 + 4;
radius = 0.1155;
R15 = rotx(-pi/2)*rotz(-pi/6) ;  % matrice di rotazione dal sdr cfr al fisso
pt15 = p_s14(:,end);                   % Punto appartenente alla crf
r15 = R15*[0 0 1]';                       % versore asse perpendicolare al piano della crf


d15  = R15*[-radius 0 -1]' + pt15;
delta15 = pt15-d15;
c15 = d15 + (delta15'*r15)*r15;
ro15 = norm(pt15-c15);
% s15 = trajPlan2(0,(pi + anglef(1))*ro15,ti15,tf15,Ts); %%%%%%%OCCHIOOOOO all angolo (180-90)/2 = 45?? -> (90+45)*ro15
s15 = trajPlan2(0,(pi/2 + pi/6)*ro15,ti15,tf15,Ts); %%%%%%%OCCHIOOOOO all angolo (180-90)/2 = 45?? -> (90+45)*ro15
cfr15 = trajCircle(r15,d15,pt15,s15,R15);

figure(1)
grid on,hold on
axis equal
view(3)
plot3(cfr15(1,:),cfr15(2,:),cfr15(3,:),'b')
hold on
plot3(c15(1),c15(2),c15(3),'ro')
plot3(d15(1),d15(2),d15(3),'r*')
plot3(pt15(1),pt15(2),pt15(3),'bx')
xlabel('X');ylabel('Y');zlabel('Z')


alpha15 = 0;
R15_1 = roty(-pi/2)*roty(-pi/2);
% R15_1 = roty(-pi/2);
T15 = shakeBeer(cfr15,c15,ro15,alpha15,R15_1);
res15 = solveIK(cfr15,T15,T30);
q15_7 = -pi/2*ones(1,length(res15));
q15_8 = zeros(1,length(res15));
res15_p = [q15_7' q15_8'];
save checkpoint15
%% Traiettoria 16 - Posa della bottiglia
Pin16 = double(subs(TE0(1:3,end),{q1,q2,q3,q4,q5,q6},{res15(end,:)}));
Pf16 = Pf2;
ti16 = tf15;
tf16 = ti16 + 3;
[p_s16,dp_s16,ddp_s16] = trajPlan2(Pin16,Pf16,ti16,tf16,Ts);
tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res15(end,:)}));
% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s16)
    R_t16(i).R = tmp(1:3,1:3);   
end
res16 = solveIK(p_s16,R_t16,T30);
% Plate
q16_7 = -pi/2*ones(1,length(res16));
q16_8 = zeros(1,length(res16));
res16_p = [q16_7' q16_8'];
save checkpoint16
%% Traiettoria 17 - rilascio bottiglia
Pin17 = [-0.3454  -0.5003   0.3306]';
Pf17 =  [-0.3454  -0.5003   0.5506]';
ti17 = tf16;
tf17 = ti17 + 2;
[p_s17,dp_s17,ddp_s17] = trajPlan2(Pin17,Pf17,ti17,tf17,Ts);


tmp = double(subs(TE0,{q1,q2,q3,q4,q5,q6},{res16(end,:)}));

% Inizializzo struttura di matrici di rotazione
for i = 1:length(p_s17)
    R_t17(i).R = tmp(1:3,1:3);   
end
res17 = solveIK(p_s17,R_t17,T30);

q17_7 = -pi/2*ones(1,length(res17));
q17_8 = zeros(1,length(res17));
res17_p = [q17_7' q17_8'];
% load('checkpoint3')
save('checkpoint17')

% Traiettoria 18 - ritorno alla configurazione iniziale
qi18 = res17(end,:);
dqi18 = zeros(1,6);
ddqi18 = zeros(1,6);
ti18 = tf17;

% Condizioni finali 
qf18 = [0 +pi/2 0 0 0 -pi/2];
dqf18 = zeros(1,6);
ddqf18 = zeros(1,6);
tf18 = ti18 + 2;

[q18_1,q18_2,q18_3,q18_4,q18_5,q18_6] = trajPlan1(qi18,qf18,ti18,tf18,dqi18,dqf18,ddqi18,ddqf18,Ts);
res18 = [q18_1', q18_2', q18_3', q18_4', q18_5', q18_6'];

% Plate
q18_7 = -pi/2*ones(1,length(res18));
q18_8 = zeros(1,length(q18_1));
res18_p = [q18_7', q18_8'];
save checkpoint18
%% Plot Robotic Toolbox
res = [res1;res2;res3;res4;res5;res6;res7;res8;res9;res10;res10_1;res11;res11_1;res12;res13;res14;res15;res16;res17;res18];
res_p = [res1_p;res2_p;res3_p;res4_p;res5_p;res6_p;res7_p;res8_p;res9_p;res10_p;res10_1_p;res11_p;res11_1_p;res12_p;res13_p;res14_p;res15_p;res16_p;res17_p;res18_p];
% res = [res1;res2;res3;res4;res5;res6;res7;res8;res13;res14;res15;res16];
% res_p = [res1_p;res2_p;res3_p;res4_p;res5_p;res6_p;res7_p;res8_p;res9_p;res10_p;res10_1_p;res11_p;res11_1_p;res12_p;res13_p;res14_p;res15_p;res16_p];
% res = res8;

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
for i = 1:20:length(res)
IRB140.plot(res(i,:));
disp(i)
pause(0.5)
end
% IRB140.plot(qf);
% run CoppeliaSimAPI