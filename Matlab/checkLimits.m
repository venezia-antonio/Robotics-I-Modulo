
function setOut = checkLimits(set)
min = deg2rad([-180 -90 -230 -200 -115 -400]);
max = deg2rad([180 110 50 200 115 400]);
flag = 0;
for i = 1:size(set,1)
    for j = 1:size(set,2)
        if(set(i,j)<=min(j) && set(i,j)>=max(j))
            flag = 1;
        end
    end
end
if (flag == 1)
    setOut = [];
    disp('No feasible set solution for ABB140')
else
    setOut = set;
end
end