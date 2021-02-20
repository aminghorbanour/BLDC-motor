function [dx] = pendulum_dyn( t,x,u,p,w )
 global g M l Io mo gr b Lm P R Vs m lG
% Planar 2link manipulator

q = x(1);
qdot = x(2);

ra = u(1);
rb = u(2);
rc = u(3);
tau = u(4);

%%

dx(1) = qdot;
dx(2) = (1/(Io+mo*gr^2))*(tau - (b*gr^2 + (Lm^2*gr^2/R)*1.5)*qdot - (M+m)*g*lG*sin(q));

dx(3) = (Vs^2/R)*[ra,rb,rc]*[ra;rb;rc] - tau*qdot;
end