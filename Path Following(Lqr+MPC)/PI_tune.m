function [G, Kp, Ki] = PI_tune(tau, ratio, desired_settling_time, tolerance)
    % Define the transfer function G
    numerator = [1 ratio];
    denominator = [tau 1 0 0]; 
    G = tf(numerator, denominator);

    % Initial guess for Kp and Ki
    Kp = 0.7;
    Ki = Kp * ratio;
    
    % Define the initial controller and closed-loop system
    C = pid(Kp, Ki);
    closedLoop = feedback(C * G, 1);
    info = stepinfo(closedLoop);
    settling_time = info.SettlingTime;
    
    % Set tolerance value for settling time
    tolerance_val = desired_settling_time * tolerance / 100;
    
    % Adjust Kp and Ki based on the settling time
    while abs(settling_time - desired_settling_time) > tolerance_val
        % Adjust Kp and Ki based on whether we need to speed up or slow down the response
        if settling_time > desired_settling_time
            Kp = Kp * 1.1;  % Increase Kp to make the system faster
            Ki = Ki * 1.1;  % Increase Ki to assist in faster error correction
        else
            Kp = Kp * 0.9;  % Decrease Kp to slow down the response
            Ki = Ki * 0.9;  % Decrease Ki to slow down integration
        end
        
        % Update the controller and closed-loop system
        C = pid(Kp, Ki);
        closedLoop = feedback(C * G, 1);
        info = stepinfo(closedLoop);
        settling_time = info.SettlingTime;
    end

    % Display the final tuned controller parameters
    fprintf('Final Tuned Kp: %.2f, Ki: %.2f\n', Kp, Ki);
    fprintf('Final Settling Time: %.2f seconds\n', settling_time);
end
