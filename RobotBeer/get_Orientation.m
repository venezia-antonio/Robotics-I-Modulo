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

