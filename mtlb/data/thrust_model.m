% https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/nozzle/
% https://www.grc.nasa.gov/www/k-12/BGP/ienzl.html
function [pe, ve, mdot] = thrust_model(mdot, Ae, To, Po, Isp)

    go  = 9.80665;
    Pa  = 101325;
    if nargin < 5
        Isp = 276;
    end
    if nargin < 4
        Po = Pa;
    end
%     if To < 2000
%         To = To*go;
%     end
    if isempty(mdot)
        % Dp = (pe - Po)
        % vx = Isp*go  
        % To = mdot vex + Dp Ae = mdot vx = mdot.Isp.go  
        mdot = To/(Isp*go);
    end
    T1 = Isp*go*mdot;
%     if Po == 0
%         pe = (To - T1)/Ae;
%     else
        pe  = abs(T1 - To)/Ae + Po;     
        ve  = (To - Ae*(pe-Po))/mdot;
%    end
    
    %mdot = 1.940;    % kg/s
    %tc = 80;         % tiempo de quema
    % T = 257.5 1 atm    
    % T  = 257.4 kgf
    % Po = 1.033 kgf/cm2  
%     pe = 0.244*go*1e4; % kgf/cm2
%     ve = 2000.56; % m/s
    % T  = 257.4 kgf
    % Po = 1.033 kgf/cm2
    % h_max = 59.8 km
    
end

