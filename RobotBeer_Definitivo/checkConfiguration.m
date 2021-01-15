function [] = checkConfiguration(jointVariables)
disp('Checking configuration...')
% Joint2
a = find(jointVariables(2,:)-pi/2<=deg2rad(-110));
b = find(jointVariables(2,:)-pi/2>=deg2rad(200));

% Joint3
c = find(jointVariables(3,:)<=deg2rad(-50));
d = find(jointVariables(3,:)>=deg2rad(280));

% Joint5
e = find(jointVariables(5,:)<=deg2rad(-120));
f = find(jointVariables(5,:)>=deg2rad(240));

if(isempty([a b c d e f]) == false)
    error('Configuration Not Allowed !');
else
    disp('Configuration Allowed')
end
end
