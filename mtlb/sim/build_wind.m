% LW  : ho, Uo, hdg ⁰ capa límite hasta 500m  
% HW  : h , U , hdg ⁰ h > 500
% gust: h1, h2, Umax
%
% addpath '../../aero'
function wind = build_wind(CL, HW, gust, qen, nfig)
    if nargin < 1 %|| isempty(CL)
        CL = []; %[10 10 0];
    end
    if nargin < 2 || isempty(HW)
        HW = [1000 0 0];
    end
    if ~ isempty(CL)
        h1  = 0:10:500; 
        UL  = wind_shear(CL(2), h1, 0.1, CL(1));
        RL  = h1*(HW(1,3)-CL(3))/h1(end) + CL(3);
    else
        h1 = [];
        UL = [];
        RL = [];
    end
    if nargin < 2
        HW = [1000 UL(end) RL(end)
              2000 0       RL(end)];
    end
    
    h2  = HW(:,1); 
    UH  = HW(:,2); 
    RH  = HW(:,3);

    hgt = [h1' ; h2 ; h2(end)+[1000 ; 2000] ; 2e5];
    hdg = [RL' ; RH ; RH(end)*[1    ; 1   ] ; 0];
    wnd = [UL' ; UH ; UH(end)*[1    ; 0   ] ; 0];

    if nargin > 2 && ~ isempty(gust)
        vg  = gust(3);
        h1  = gust(1);
        h2  = gust(2);
        HG  = h1:10:h2;
        VG  = cosine_gust(HG-h1)*vg;        
        
        k   = find(hgt > h1,1) - 1;
        RG  = hdg(k) * ones(size(HG));
        Vo  = interp1(hgt, wnd, HG); 
        VG = VG + Vo;
        
        hgt = [hgt(1:k) ; HG'; hgt(k+1:end)];
        hdg = [hdg(1:k) ; RG'; hdg(k+1:end)];
        wnd = [wnd(1:k) ; VG'; wnd(k+1:end)];

    end
    hdg = hdg - 180;
    sn  = sind(hdg);
    cs  = cosd(hdg);
    h = hgt;
    u = wnd .* cs; % norte
    v = wnd .* sn; % este 
    
    wind.h = h;
    wind.u = u;
    wind.v = v;
    wind.w = zeros(size(v));
    if nargin > 4 && ~isempty(nfig) && nfig > 0 
        
        figure(nfig); clf;

        h(end) = [];
        u(end) = [];
        v(end) = [];
        subplot(121);
        Z = zeros(size(h));
        quiver3(Z, Z, h, v, u, Z, 'ShowArrowHead', 'off');
        xlabel('E');
        ylabel('N');
        zlabel('h');
        grid on;

        h = 0:10:h(end);
        u = interp1(wind.h, wind.u, h, 'pchip','extrap');  
        v = interp1(wind.h, wind.v, h, 'pchip','extrap');  
        
        subplot(322);
        plot(u*3.6, h);
        hold on;
        plot(wind.u(1:end-1)*3.6, wind.h(1:end-1), '.');
        xlabel('N [km/h]');
        ylabel('h [m]');
        grid on;
        
        subplot(324);
        plot(v*3.6, h);
        hold on;
        plot(wind.v(1:end-1)*3.6, wind.h(1:end-1), '.');
        xlabel('E [km/h]');
        ylabel('h [m]');
        grid on;

        subplot(326);
        plot(v*3.6, u*3.6);
        hold on;
        plot(wind.v(1:end-1)*3.6, wind.u(1:end-1)*3.6, '.');
        xlabel('E [km/h]');
        ylabel('N [km/h]');
        grid on;
    end
    
    if nargin > 3 && ~isempty(qen) 
        for k=1:size(wind.h,1)
            Wn = [wind.u(k) ; wind.v(k) ; wind.w(k) ];
            We = qen.trnsf(Wn); 
            wind.u(k) = We(1);
            wind.v(k) = We(2);
            wind.w(k) = We(3);
        end
    end  

end

