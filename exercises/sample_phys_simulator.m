% Initial Conditions
y = 10;          % Initial height (meters)
v = 0;           % Initial velocity (m/s)
m = 1;           % Mass (kg)
g = -9.81;       % Gravity (m/s^2)
dt = 0.01;       % Time step (seconds)
t_end = 4;       % Simulation duration

% Simulation Loop
for t = 0:dt:t_end
    % Physics Equations
    F = m * g;           % F = ma
    a = F / m;           % Acceleration
    v = v + a * dt;      % Update Velocity
    y = y + v * dt;      % Update Position

    % Collision Detection with Ground
    if y <= 0
        y = 0;
        v = -v * 0.8;    % Bounce with 20% energy loss
    end

    % Real-time Plotting
    plot(t, y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    xlim([0 t_end]); ylim([0 12]);
    grid on;
    hold on;
    pause(0.01);
end
