function  a_integrator = Velocity_integrator(t_i,Vx,Vdes)
     syms a_sym(t)
     a_sym(t) = int (Vdes-Vx, 0, t);
     %Get the value 
     a_integrator = double (a_sym(t_i));
end 
