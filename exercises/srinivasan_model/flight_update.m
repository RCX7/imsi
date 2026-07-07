function derivatives = flight_dynamics(X, Xdot, Y, Ydot, F, Ts)
    derivatives = [Xdot, 0, Ydot, -g];
end