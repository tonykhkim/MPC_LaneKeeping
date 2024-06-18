function fplotState(hAxis, state_array, idx)
    % Plot the state array on the specified axis
    switch idx
        case 2
            str = "Y";
        case 3
            str = "vy";
        case 4
            str = "yaw";
        case 5
            str = "yawrate";
        otherwise

    end
    line(hAxis, 1:length(state_array(:, idx)), state_array(:, idx), ...
         'DisplayName', str);
    legend(hAxis, 'show');
end
