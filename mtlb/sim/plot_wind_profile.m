function plot_wind_profile(wnd, h, ln) %, clr)
    if nargin < 3
        ln = 2;
    end
%     if nargin < 4
%         clr = [0 1 1 ; 1 0 1 ];
%     end
    u = interp1(wnd.h, wnd.u, h, 'pchip','extrap');
    if isfield(wnd, 'v') 
        v = interp1(wnd.h, wnd.v, h, 'pchip','extrap');
    else
        v = zeros(size(u));
    end
    plot(u, h, 'LineWidth', ln); %, 'Color', clr(1,:)); 
    hold on;
    plot(v, h, 'LineWidth', ln); %, 'Color', clr(2,:)); 
    grid on; 
    ylabel('h [m]');
    xlabel('viento [m/s]');
    legend('N', 'E', 'AutoUpdate', 'off');
    plot([0 0], [h(1) h(end)], 'k');
end

