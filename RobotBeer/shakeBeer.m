function T = shakeBeer(cfr,c,ro,alpha,R)
% alpha = sym(5*pi/12);           % Angolo di basculazione
h = double(tan(alpha)*ro);       % Altezza del cono 
vertice = [c(1);c(2);c(3)+h];        % Vertice del cono

% Determino il versore che definisce la retta orientata al variare di gamma
for i = 1:length(cfr)
    versore(:,i) = (vertice - cfr(:,i))./norm(vertice - cfr(:,i));
    assi(:,i) = versore(:,i);
end


% Determino i coseni direttori della retta orientata
% Effettuo il prodotto scalare del versore della retta con i versori degli
% assi X,Y e Z
Xver = [1 0 0]';
Yver = [0 1 0]';
Zver = [0 0 1]';

% Inizializzo il vettore contenente i coseni direttori della retta
cos_dir = zeros(3,length(cfr));

for i = 1:length(cfr)
    cos_dir(1,i) = dot(assi(:,i),Xver);
    cos_dir(2,i) = dot(assi(:,i),Yver);
    cos_dir(3,i) = dot(assi(:,i),Zver);
end


% Inizializzo il vettore dei versori che definisco sdr levogiro
axis_x = zeros(3,length(cfr));
axis_y = zeros(3,length(cfr));
axis_z = zeros(3,length(cfr));


for i = 1:length(cfr)
u = cos_dir(:,i); % =assi
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
    T(i).R = [axis_x(:,i) axis_y(:,i) axis_z(:,i)]*R;
end 

end