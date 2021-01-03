clear all
h = 1;
r = h;
gamma = 0:0.1:2*pi;


z = h*ones(1,length(gamma));

x = 0*ones(1,length(gamma));
y = 0*ones(1,length(gamma));
quota = 1;
quota = quota*ones(1,length(gamma));
circonferenza = [x+r*cos(gamma);y+r*sin(gamma);quota];
vertice = [x;y;quota+r];

for i = 1:length(vertice)
versore(:,i) = (vertice(:,i) - circonferenza(:,i))./norm(vertice(:,i) - circonferenza(:,i));
assi(:,i) = versore(:,i);
end


% figure(1)
% view(3)
% grid on,hold on
% plot3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),'r')
% plot3(vertice(1,:),vertice(2,:),vertice(3,:),'ro')
% quiver3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),assi(1,:),assi(2,:),assi(3,:),sqrt(r^2+h^2));


for i = 1:length(versore)-1
axang1 = [versore(1,i) versore(2,i) versore(3,i) 0];
axang2 = [versore(1,i+1) versore(2,i+1) versore(3,i+1) 1e-05];

R1 = axang2rotm(axang1);
R2 = axang2rotm(axang2);
R = R1'*R2;
% axangle = rotm2axang(R);
% % theta(i) = axangle(end);
% Ri = axang2rotm(axangle);

angles(i,:) = tr2eul(R);
end


theta = [0 0 0 0 0 0];
L3=0.380;
L4=0.046;
L(1) = Link([theta(4),L3,0,-(pi)/2],'standard');
L(2) = Link([theta(5),0,0,(pi)/2],'standard');
L(3) = Link([theta(6),L4,0,0],'standard');
IRB140 = SerialLink(L,'name','IRB140');
figure(2)
hold on,grid on
view(3)
xlim([-0.3 0.3]);ylim([-0.3 0.3]);zlim([0 1]);
for i = 1:length(angles)
    IRB140.plot(angles(i,:))
end






