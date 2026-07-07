rng(42);
% constants / constraints
g = 9.81;
lmax = 1;   % leg length (m)
m = 70;     % kg
N = 10;     % segments of force curve (endt/N = period of linear force)

dt = 0.01;  % computation segments
endt = 10;  % end time (s)

V = 0.5;    % non dimensional velocity constraint (m/s)
D = 0.8;    % non dimensional step length constraint (m)

% starting conditions (arbitrary guesses)
xc = 0;
x0 = 0.2;
y0 = 0.9;
xdot0 = 0.5;    % moving forward at 0.5 m/s
ydot0 = 0;
ts = 0.6;       % step period

% state variables
x = x0;
y = y0;
xdot = xdot0;   % x velocity
ydot = ydot0;   % y velocity


% non dimensionalize
rootglmax = sqrt(g * lmax);
X = x / lmax;
Xdot = xdot0 / rootglmax;
Y = y / lmax;
Ydot = ydot / rootglmax;
Ts = ts / rootglmax;


% set up force and state vector
F_guess = ones(N+1, 1) * 1.0;
z0 = [X; Xdot; Y; Ydot; F_guess; Ts];

disp("--- starting conditions: ---");
fprintf("mass position (x, y): (%f, %f)\n", x, y);
fprintf("mass velcoity (x, y): (%f, %f)\n", xdot, ydot);


[T_stance, S_stance] = ode45(@stance_update, [0, Ts], z, options);
