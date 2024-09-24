% Ejemplo: dat= msdat_read('../../../DATCOM/T2-70'); show_datcom_coefs(dat)
function show_datcom_coef(coef, F, alfa, Fn)

    if nargin < 6
        Fn = F;
    end
    if nargin < 3 || isempty(alfa)
        alfa = [0:1:5 7 9 12 20 45 60 75 90];
    end 
    alfa = alfa * pi/180;
    NP = 6;
    %% 
    mch  = [0:0.1:0.6 0.7:0.025:1.5 1.6:0.1:5];
    slp  = (0:1:45) * pi/180;
    beta = [0 1 2 4 8 12] * pi/180;
    mach = [0.2 0.5 0.9 1 1.1 2];
    assert(length(beta) == length(mach));
    for k=NP:-1:1
        FigH = figure(k); 
        clf
        set(FigH, 'NumberTitle', 'off', 'Name', sprintf('β=%0.2g⁰ / M=%0.2g', beta(k)*180/pi, mach(k)));   

        subplot(121);
        plot_beta(beta(k), coef, alfa, mch, F, Fn);
        
        subplot(122);
        plot_mach(mach(k), coef, alfa, slp, F, Fn);
        plot_mach_data(mach(k), coef, F, Fn);
    end
end

    
% [mach x alfa x beta]
function plot_beta(slp, coef, alfa, mach, F, f)
    r2d = 180/pi;
    [A,B] = meshgrid(alfa*r2d, mach);
    [N,M] = meshgrid(coef.alpha*r2d, coef.mach);
    
    F = getfield(coef, F);
    C = interp3(coef.alpha, coef.mach, coef.beta, F, alfa, mach, slp, 'spline');  
    mesh(A, B, C); zlabel(f); xlabel('α'); ylabel('M'); 
    zlim([min(min(C)) max(max(C))]);
    hold on;
    C = interp3(coef.alpha, coef.mach, coef.beta, F, coef.alpha, coef.mach, slp);  
    plot3(N, M, C, '.'); 
    xlim([alfa(1) alfa(end)]*r2d);
    ylim([mach(1) mach(end)]);
    grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end
% Vq = interp3(X,Y,Z,V,Xq,Yq,Zq)
% X=1:n, Y=1:m, Z=1:p, [n,m,p] where [m,n,p] = size(V)
function plot_mach(mch, coef, alfa, beta, Fn, f)
    F = getfield(coef, Fn);    

    r2d = 180/pi;
    [A,B] = meshgrid(beta, alfa);
    [idx, frc] = find_mach(coef, mch);
    C = interp_M(coef, F, A, B, idx, frc);
    mesh(A*r2d, B*r2d, C); zlabel(f); xlabel('α'); ylabel('β'); 
    zlim([min(min(C)) max(max(C))]);
    hold on;

    C1 = interp_M(coef, F, 30/r2d, 0, idx, frc);
    plot3(30, 0, C1, 'sk');
    
    xlim([alfa(1) alfa(end)]*r2d);
    ylim([beta(1) beta(end)]*r2d);
    grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end
function plot_mach_data(mch, coef, Fn, f)
    r2d = 180/pi;
    [N,M] = meshgrid(coef.beta*r2d, coef.alpha*r2d);
    F = getfield(coef, Fn);    
    [idx, frc] = find_mach(coef, mch);
    C = squeeze(F(idx,:,:));
    plot3(M, N, C, '-x'); 
    hold on;
    
%     xlim([coef.alpha(1) coef.alpha(end)]*r2d);
%     ylim([coef.beta(1) coef.beta(end)]*r2d);
%     zlim([min(min(C)) max(max(C))]);
% 
%     grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end



