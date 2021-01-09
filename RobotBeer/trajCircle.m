function cfr = trajCircle(r,d,Pi,s)
%     r = [0 0 1]';     % versore asse della cfr
%     d = [0 0 2]';     % coordinata punto sull asse
%     Pi = [0.3 0 0]';  % vettore posizione punto pi della cfr
%     
    delta = Pi-d;
    c = d + (delta'*r)*r;
    ro = norm(Pi-c);


    for i = 1:length(s)
      cfr(:,i) = rotz(-pi)*[ro*cos(s(i)/ro); ro*sin(s(i)/ro);0] + c;
    end
    
    figure(1)
    grid on,hold on
    axis equal
    view(3)
    plot3(cfr(1,:),cfr(2,:),cfr(3,:))
    hold on
    plot3(c(1),c(2),c(3),'ro')
end