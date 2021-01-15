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