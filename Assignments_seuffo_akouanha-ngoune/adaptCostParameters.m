function [Q, R, Pf] = adaptCostParameters(V_update, Vx)
    % Increase positional accuracy weight if V_update > baselineSpeed
    speedFactor = max(1, V_update /(Vx));
    Q = diag([8e3 * speedFactor, 0, 0, 6e3 * speedFactor]); % Increase with speed
    %fixed control input cost
    R = 2e5; 
    % Terminal cost similar to running cost
    Pf = diag([1, 100, 1 , 6000]);
end
