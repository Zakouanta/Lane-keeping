close all, clear all , clc

run('num_parameters.m')
[x, y, psidot_des_mpc, psi_des_mpc, s] = reference_mpc_left_turn(Vx, L_straight, L_straight, N_points);
figure;
plot(x, y, 'r');
title('Reference Path');
xlabel('X reference');
ylabel('Y reference');
grid on;

