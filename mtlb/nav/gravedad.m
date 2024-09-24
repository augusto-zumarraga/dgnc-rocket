function [g, R] = gravedad(h)
    Re = 6375.4e3;
    R  = Re + h;
    g  = 9.80665 * (Re./R).^2;
end