function circonferenza = trajCircle()
r = [0 0 1]'; % versore asse della cfr
d = [0 0  2]'; % coordinata punto sull asse
Pi = [0.3 0 0]'; % vettore posizione punto pi della cfr
delta = Pi-d;
c = d + (delta'*r)*r;

ro = norm(Pi-c);
i = 1;
for s = 0:0.1:2*pi*ro
circonferenza(:,i) = [ro*cos(s/ro);
                      ro*sin(s/ro);
                           0       ] + c;
        
  i = i+1;
end
figure(1)
grid on,hold on
axis equal
view(3)
plot3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:))
end