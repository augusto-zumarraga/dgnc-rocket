% --------------------------------- masa fija
% mass().xcg:
% mass().m :
% mass().J :
% --------------------------------- propelente
% mass().xo:
% mass().m :
% mass().l :
% mass().d :

function mass = mass_props(data, frac)
    Z = zeros(size(frac));

    mass.m   = Z;
    mass.xcg = Z;
    mass.Ix  = Z;
    mass.Iy  = Z;
    mass.Iz  = Z;

    for k=1:length(frac)
        xcg = 0;
        f   = frac(k);
        assert(f >= 0);
        assert(f <= 1);
        for n=1:length(data)
            if data(n).xcg > 0
                m  = data(n).m;
                x  = data(n).xcg;
                Ix = data(n).J(1);
                Iy = data(n).J(2) + m*x^2;
                Iz = data(n).J(3) + m*x^2;
            else
                [m, x, Ix, Iy, Iz] = tank(data(n), f);
            end
            mass.m (k) = mass.m (k) + m;
            mass.Ix(k) = mass.Ix(k) + Ix;
            mass.Iy(k) = mass.Iy(k) + Iy;
            mass.Iz(k) = mass.Iz(k) + Iz;
            xcg = xcg + m * x;
        end
        m = mass.m(k); 
        xcg = xcg / m; 
        mass.xcg(k) = xcg;
        Io  = m * xcg^2;
        mass.Iy(k) = mass.Iy(k) - Io;
        mass.Iz(k) = mass.Iz(k) - Io;
    end
end

function [m, x, Ix, Iy, Iz] = tank(tk, f) 
    m  = tk.m * f;
    l  = tk.l * f; 
    x  = tk.xo - l*f/2; 

	assert(isfield(tk, 'd'));
    Ix = m * tk.d^2 / 8; 
    Iy = m * l^2 / 12;    % al medio
    Iy = Iy + m * x^2;
    Iz = Iy;    
end

