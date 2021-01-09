%% Compute Inverse Kinematics for each time instant 
% Change the output in order to use another solution of IK for ABB IRB 140

% p_t is the position vector at time instant t (3 x number of point of
% trajectory)

% R_t is the orientation matrix at time instant t (it's a structure)
function set2 = solveIK(p_t,R_t,T30)
    h = waitbar(0,'Please wait...');
    for i = 1:length(p_t)
        % Impostare posizione e orientamento desiderato
        p = [p_t(1,i) p_t(2,i) p_t(3,i)]';
        % R = double(subs(TE0(1:3,1:3),{q1,q2,q3,q4,q5,q6},{qf}));
        Tik = [R_t(i).R p;0 0 0 1];
        [s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
        set1(i,:) = s1;
        set2(i,:) = s2;
        set3(i,:) = s3;
        set4(i,:) = s4;
        set5(i,:) = s5;
        set6(i,:) = s6;
        set7(i,:) = s7;
        set8(i,:) = s8;
        waitbar(i/length(p_t),h)
    end
    close(h);
end