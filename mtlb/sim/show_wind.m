function wnd = show_wind(h, param, qen, nfig)

    if isrow(h)
        h = h';
    end
    wnd = zeros(length(h),3);
    for k=1:length(h)
        z3 = [0;0;0];
        wnd(k,:) = wind_3D(h(k), z3, z3, qen', qen, param);
    end
    if nargin > 3 && nfig > 0 
        
        figure(nfig); 
        clf;
        
        u = wnd(:,1)*3.6;
        v = wnd(:,2)*3.6;
        w = wnd(:,3)*3.6;
        subplot(121);
        Z = zeros(size(h));
        quiver3(Z, Z, h, v, u, w, 'ShowArrowHead', 'off');
        xlabel('E');
        ylabel('N');
        zlabel('h');
        grid on;
        
        %%
        subplot(422);
        plot(u, h);
        xlabel('N [km/h]');
        ylabel('h [m]');
        grid on;
        
        subplot(424);
        plot(v, h);
        xlabel('E [km/h]');
        ylabel('h [m]');
        grid on;

        subplot(426);
        plot(w, h);
        xlabel('U [km/h]');
        ylabel('h [m]');
        grid on;
        
        subplot(428);
        plot(v, u);
        xlabel('E [km/h]');
        ylabel('N [km/h]');
        grid on;
    end    

end

