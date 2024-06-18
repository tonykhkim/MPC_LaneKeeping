function value = fgenSineWave()
    % Persistent variables retain their values between calls to the function
    persistent t lastTime amplitude frequency;

    % Initialize variables if they are empty
    if isempty(t)
        t = 0;              % Initial time
        lastTime = 0;       % Last time the function was called
        amplitude = 1*pi/180;      % Amplitude of the sine wave
        frequency = 0.3;    % Frequency of the sine wave, in Hz
    end

    % Time interval between function calls
    timeInterval = 0.01;   % 0.01 seconds

    % Update the time only if 0.01 seconds have passed
    currentTime = tic;
    if currentTime - lastTime >= timeInterval
        lastTime = currentTime;
        t = t + timeInterval;

        % Calculate the sine wave value
        value = amplitude * sin(2 * pi * frequency * t);

        % For demonstration, printing the value
        % fprintf('Time: %.2f, Sine Value: %.2f\n', t, value);
    end
end