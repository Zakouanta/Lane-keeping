function u = deltaf(amplitude, frequency)

u = @(t) deg2rad(amplitude * sin(2 * pi * frequency * t));

end 
