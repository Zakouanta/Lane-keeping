function xdotPP = xdotPP(A,B,x,B_d,w,Kpp,delta_ff)
xdotPP = (A - B*Kpp)*x + B*delta_ff + B_d*w;
end