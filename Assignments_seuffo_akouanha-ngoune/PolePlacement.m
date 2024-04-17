function Kpp = PolePlacement(deltap,omega_n,A,B)
    p1 = -deltap*omega_n+1i*omega_n*sqrt(1-deltap^2);
    p2= -deltap*omega_n-1i*omega_n*sqrt(1-deltap^2);
    P = [p1 p2 -10  -100];
    Kpp = place(A,B,P);
end 
