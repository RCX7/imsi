function derivatives = stance_update(X, Xdot, Y, Ydot, F, Ts)
    XC = 0;
    
    Xddot = (F * (X-Xc))/l;
    Yddot = -mg + FY/l;

    derivatives = [Xdot; Xddot; Ydot; Yddot; F; Ts];
end