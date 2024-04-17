function u = delta(t, frequency)
    % Calculate steering angle as a function of time
    u = deg2rad(30 * sin(2 * pi * frequency * t));
end