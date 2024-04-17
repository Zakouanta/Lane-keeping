function [omega_n , S] = specifications(deltap,deviation,zeta)

    S = 100 * exp ((-pi*deltap)/sqrt(1-deltap^2));
    
    % Check the respect of the overshoot constraint
    if S >= deviation
        disp('Error! The controller design does not satisfy overshoot constraint.');
    end
    
    omega_n = 4/(deltap*zeta);     % natural frequency
    
end 