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
