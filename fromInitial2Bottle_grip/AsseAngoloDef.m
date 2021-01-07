clear all
run createRobot
%% Generazione circonferenza e assi cono
r = 0.01;                       % Raggio della circonferenza basculante
alpha = sym(5*pi/12);           % Angolo di basculazione
h = double(tan(alpha)*r);       % Altezza del cono 
gamma = linspace(0,2*pi,64);    % Angolo per spazzare la circonferenza
z = h*ones(1,length(gamma));    % Coordinata lungo z 
x = -0.2954*ones(1,length(gamma));  % Coordinata lungo x
y = -0.5*ones(1,length(gamma));  % Coordinata lungo y
quota1 = 0.5506;                   % Posizione della circonferenza lungo z
quota = quota1*ones(1,length(gamma));
circonferenza = [x+r*cos(gamma);y+r*sin(gamma);quota];
vertice = [x;y;quota+r];

% Determino il versore che definisce la retta orientata al variare di gamma
for i = 1:length(gamma)
versore(:,i) = (vertice(:,i) - circonferenza(:,i))./norm(vertice(:,i) - circonferenza(:,i));
assi(:,i) = versore(:,i);
end

% Plot circonferenza e rette al variare di gamma
figure(1)
view(3)
grid on,hold on
plot3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),'r')
plot3(vertice(1,:),vertice(2,:),vertice(3,:),'ro')
quiver3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),assi(1,:),assi(2,:),assi(3,:),sqrt(r^2+h^2));
hold on
plot3(circonferenza(1,1),circonferenza(2,1),circonferenza(3,1),'bx')
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Determino i coseni direttori della retta orientata
% Effettuo il prodotto scalare del versore della retta con i versori degli
% assi X,Y e Z
Xver = [1 0 0]';
Yver = [0 1 0]';
Zver = [0 0 1]';

% Inizializzo il vettore contenente i coseni direttori della retta
cos_dir = zeros(3,length(gamma));

for i = 1:length(gamma)
    cos_dir(1,i) = dot(assi(:,i),Xver);
    cos_dir(2,i) = dot(assi(:,i),Yver);
    cos_dir(3,i) = dot(assi(:,i),Zver);
end


% Inizializzo il vettore dei versori che definisco sdr levogiro
axis_x = zeros(3,length(gamma));
axis_y = zeros(3,length(gamma));
axis_z = zeros(3,length(gamma));


for i = 1:length(gamma)
u = cos_dir(:,i);
v = [-cos_dir(3,i),0,cos_dir(1,i)]';
w = cross(v,u);

% Normalizzo i vettori per ottenere i versori 
u = u/norm(u);
v = v/norm(v);
w = w/norm(w);
axis_x(:,i) = v; 
axis_y(:,i) = u;
axis_z(:,i) = w;
end

% Inizializzo la matrice di trasfromazione
T = struct([]);
for i = 1:length(axis_x)
    T(i).R = [axis_x(:,i) axis_y(:,i) axis_z(:,i)];
end 


% Verifico che sia una matrice di rotazione levogira
for i = 1:length(axis_x)
    disp(det(T(i).R))
end


%% Inverse Kinematics
qw = waitbar(0,'Please wait...');

for i = 1:length(axis_x)-1
    p = [circonferenza(1,i),circonferenza(2,i),circonferenza(3,i)]';
    R = T(i).R;
    Tik = [R p;0 0 0 1];
    [s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
    set_1(i,:) = s1;
    set_2(i,:) = s2;
    set_3(i,:) = s3;
    set_4(i,:) = s4;
    set_5(i,:) = s5;
    set_6(i,:) = s6;
    set_7(i,:) = s7;
    set_8(i,:) = s8;
    h = waitbar(i/length(axis_x),qw);
end
close(h)
%%
close all
figure(2)
hold on,grid on
view(3)
plot3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),'r');
% quiver3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),assi(1,:),assi(2,:),assi(3,:),sqrt(r^2+h^2));
xlim([-1 1]);ylim([-1 1]);zlim([-0.2 1.5]);
for i = 1:length(circonferenza)-1
 IRB140.plot(set_2(i,:));
pause(0.5)
% IRB140.plot([0 pi/2 0 angles(i,1) angles(i,2) angles(i,3)]);
end
