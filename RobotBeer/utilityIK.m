
Pin4 = [-0.3454  -0.5003   0.5506]';

p = [-0.2794 -0.5013 0.0951]';
R = double(IRB140.fkine(res3(end,:)));
R = R(1:3,1:3)
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
IRB140.plot(s2)