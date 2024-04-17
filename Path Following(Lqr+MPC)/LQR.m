function [Q, K_lqr, S, CLP] = LQR(A, B)
    Q = 0.3;
    R = 30;
    N = 0;
    [K_lqr,S,CLP] = lqr(A, B, Q, R, N);
end