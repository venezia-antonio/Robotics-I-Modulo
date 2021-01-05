
p = [-0.3454  -0.5003   0.5506]';
R = eye(3)*roty(-pi/2)*rotz(-pi/2);
Tik = [R p;0 0 0 1];
[s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
IRB140.plot(s2)