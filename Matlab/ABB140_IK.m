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
Le = 0.03;

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
r = 0.6;
j = 1;
for i = 0:0.1:2*pi
    x(j) = r*cos(i);
    y(j) = r*sin(i);
    z(j) = 0.5;
    j = j+1;
end
for i = 1:length(x)
% Impostare posizione e orientamento desiderato
p = [x(i) y(i) z(i)]';
R = eul2r([0 0 0]);
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
%IK = double(subs(TE0(1:3,end),{q1,q2,q3,q4,q5,q6},s1));
set1(i,:) = s1;
set2(i,:) = s2;
set3(i,:) = s3;
set4(i,:) = s4;
set5(i,:) = s5;
set6(i,:) = s6;
set7(i,:) = s7;
set8(i,:) = s8;
% legend('Target','IK Solution')
% s = s1; % cambiare per vedere le soluzioni s1-s8
% value = double(subs(TE0,{q1,q2,q3,q4,q5,q6},s))
end
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
%% Function
function [s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30)
% Inverse Kinematics - eventualmente aggiungere l'altra soluzione del polso
[q11,q12] = get_q1(Tik);
[q31,q32] = get_q3(q11,Tik);
[q21,~,~,~] = get_q2(q11,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q21,q31);
s1 = double([q11,q21,q31,q41,q51,q61]);

[q31,q32] = get_q3(q11,Tik);
[~,~,q23,~] = get_q2(q11,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q23,q32);
s2 = double([q11,q23,q32,q41,q51,q61]);

[q31,q32] = get_q3(q12,Tik);
[~,q22,~,~] = get_q2(q12,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q22,q31);
s3 = double([q12,q22,q31,q41,q51,q61]);

[q31,q32] = get_q3(q12,Tik);
[~,~,~,q24] = get_q2(q12,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q24,q32);
s4 = double([q12,q24,q32,q41,q51,q61]);

[q31,q32] = get_q3(q12,Tik);
[~,q22,~,~] = get_q2(q12,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q22,q31);
s5 = double([q12,q22,q31,q41,q51,q61]);

[q31,q32] = get_q3(q12,Tik);
[~,~,~,q24] = get_q2(q12,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q24,q32);
s6 = double([q12,q24,q32,q41,q51,q61]);

[q31,q32] = get_q3(q12,Tik);
[~,q21,~,~] = get_q2(q12,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q21,q31);
s7 = double([q12,q21,q31,q41,q51,q61]);

[q31,q32] = get_q3(q11,Tik);
[~,~,q23,~] = get_q2(q11,q31,q32,Tik);
[q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q23,q32);
s8 = double([q11,q23,q32,q41,q51,q61]);
end
%Teta1
function [q11,q12] = get_q1(Tik)
L4=0.065;Le=0.03;
W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
Wx = W(1);Wy = W(2); Wz = W(3);
q11 = atan2(Wy,Wx);
if(Wy>=0)
    q12 = atan2(Wy,Wx) - sym(pi);
else
    q12 = atan2(Wy,Wx) + sym(pi);
end
end

% Teta3
function [q31,q32] = get_q3(q1,Tik)
L0=0.07;L1=0.3520;L2=0.360;L3=0.380;L4=0.065;Le=0.03;
W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
Wx = W(1);Wy = W(2); Wz = W(3);
r_2 = (Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2;
s_2 = (Wz - L1)^2;

h_2 = r_2 + s_2;
cosq3 = (h_2 - L2^2 - L3^2)/(2*L2*L3);
sinq3 = [sqrt(1-cosq3^2) -sqrt(1-cosq3^2)];

q31 = (atan2(sinq3(1),cosq3))+ sym(pi)/2;
q32 = (atan2(sinq3(2),cosq3))+ sym(pi)/2;

end



% Teta2
function [q21,q22,q23,q24] = get_q2(q1,q31,q32,Tik)
L0=0.07;L1=0.3520;L4=0.065;L2=0.360;L3=0.380;Le=0.03;

W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
Wx = W(1);Wy = W(2); Wz = W(3);
r(1) = sqrt((Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2);
r(2) = -sqrt((Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2);
s = (Wz - L1);
psi = (q31 - sym(pi)/2);
q21 = atan2(s,r(1)) - atan2(L3*sin(psi),L3*cos(psi)+L2);
q22 = atan2(s,r(2)) - atan2(L3*sin(psi),L3*cos(psi)+L2);
psi1 = (q32 - sym(pi)/2);
q23 = atan2(s,r(1)) - atan2(L3*sin(psi1),L3*cos(psi1)+L2);
q24 = atan2(s,r(2)) - atan2(L3*sin(psi1),L3*cos(psi1)+L2);
end
% Teta4 - Teta5 - Teta6
function [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q22,q33)
syms q1 q2 q3
R30 = double(subs(T30(1:3,1:3),{q1,q2,q3},{q11,q22,q33}));
R63 = R30'*Tik(1:3,1:3);

q41 = atan2(R63(2,3),R63(1,3));
q51 = atan2(sqrt(R63(1,3)^2+R63(2,3)^2),R63(3,3));
q61 = atan2(R63(3,2),-R63(3,1));

q42 = atan2(-R63(2,3),-R63(1,3));
q52 = atan2(-sqrt(R63(1,3)^2+R63(2,3)^2),R63(3,3));
q62 = atan2(-R63(3,2),R63(3,1));
end



function dh = DH(theta_i,d_i,link,a_i)
    dh = [cos(theta_i) -sin(theta_i)*cos(a_i)   sin(theta_i)*sin(a_i)     link*cos(theta_i);
          sin(theta_i)  cos(theta_i)*cos(a_i)   -cos(theta_i)*sin(a_i)     link*sin(theta_i);
               0         sin(a_i)                   cos(a_i)                        d_i;
               0             0                          0                           1       ];
end