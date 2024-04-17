function PI = PI(t,Vx,Vdes,Kp,Ki)
  a_integrator = Velocity_integrator(t,Vx,Vdes);
  PI = -Kp*(Vdes-Vx) - Ki*a_integrator;
end 