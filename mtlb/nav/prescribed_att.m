function [q, e] = prescribed_att(ts, t, eul, nfig)

    if nargin < 4
        nfig = 0;
    end
   
    tk = 0:ts:t(end);
    r  = interp1(t, eul(:,1), tk, 'pchip');
    p  = interp1(t, eul(:,2), tk, 'pchip');
    y  = interp1(t, eul(:,3), tk, 'pchip');
    e  = [r' p' y']; 
    q  = quaternion.from_euler(r, p, y);
    if nfig > 0
        e = quaternion.to_euler(q);
        figure(nfig);
        clf
        for k=1:3
            subplot(4,1,k);
            hold on;
            t = (0:length(q)-1)*ts;
            plot(t , e(:,k)*180/pi, 'LineWidth', 2);
            grid on;
        end
        subplot(414);
        plot(t , q, 'LineWidth', 2);
        legend('r', 'x', 'y', 'z');
        grid on;
    end
   
end

