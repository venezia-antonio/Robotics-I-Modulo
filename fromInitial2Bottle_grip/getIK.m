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


% min = [-180 -90 -230 -200 -115 -400];
% max = [180 110 50 200 115 400];
% 
% if(s1<=min && s1>=max)
%     disp('S1 - Configuration Not Allowed')
% end
% if(s2<=min && s2>=max)
%     disp('S2 - Configuration Not Allowed')
% end
% if(s3<=min && s3>=max)
%     disp('S3 - Configuration Not Allowed')
% end
% if(s4<=min && s4>=max)
%     disp('S4 - Configuration Not Allowed')
% end
% if(s5<=min && s5>=max)
%     disp('S5 - Configuration Not Allowed')
% end
% if(s6<=min && s6>=max)
%     disp('S6 - Configuration Not Allowed')
% end
% if(s7<=min && s7>=max)
%     disp('S7 - Configuration Not Allowed')
% end
% if(s8<=min && s8>=max)
%     disp('S8 - Configuration Not Allowed')
% end
end
