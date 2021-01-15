


p = [-0.2129 -0.5003 0.3231]';
R = double(IRB140.fkine(res3(end,:)));
R = R(1:3,1:3)*rotx(pi/2)*roty(pi/2)
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
IRB140.plot(s2)