function xdot = xdot(A,B,x,u,B_d,w)
xdot = A*x+B*u+B_d*w;
end