function delta_ff = feedforward(m, Vx, lf, lr, cf, cr, r, Kpp)
    L = lf + lr;
    delta_ff= m * Vx^2 / (r*L) * (lr / (2*cf) - lf / (2*cr) + lf / (2*cr) * Kpp(3)) + L/r - lr / r*Kpp(3);
end