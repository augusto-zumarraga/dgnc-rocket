% Ejemplo: dat= msdat_read('../../../DATCOM/T2-70'); show_datcom_coefs(dat)
function show_datcom_coefs_3(coef, mach, alfa, beta, vs_mach, wind_axes)

    if nargin < 5 || isempty(vs_mach)
        vs_mach = true;
    end
    if nargin < 6 
        wind_axes = true;
    end
    if nargin < 2 || isempty(mach)
        if vs_mach
            mach = [0:0.1:0.6 0.7:0.025:1.5 1.6:0.1:5];
        else
            mach = [0.4 0.8 1 1.2 1.5 2 5];
        end
    end 
    if nargin < 3 || isempty(alfa)
        alfa = [0:1:5 7 9 12 22 28 34 45 60] * pi/180;
    end 
    if nargin < 4 || isempty(beta)
        if vs_mach
            beta = (0:2:10) * pi/180;
        else
            beta = (0:1:45) * pi/180;
        end
    else
        beta = beta * pi/180;
    end 
       
    if vs_mach
        %[A,M] = meshgrid(alfa, mach);
        for k=1:length(beta) 

            slp = beta(k);

            FigH = figure(k); 
            clf
            set(FigH, 'NumberTitle', 'off', 'Name', sprintf('β=%0.2g⁰', slp*180/pi));     
            if wind_axes
            subplot(2,3,1); plot_beta(slp, coef, alfa, mach, 'cl', 'c_L');
            subplot(2,3,3); plot_beta(slp, coef, alfa, mach, 'cd', 'c_D');
            else
            subplot(2,3,1); plot_beta(slp, coef, alfa, mach, 'CX', 'c_x');
            subplot(2,3,3); plot_beta(slp, coef, alfa, mach, 'CZ', 'c_z');
            end
            subplot(2,3,2); plot_beta(slp, coef, alfa, mach, 'CY', 'c_y');
            subplot(2,3,4); plot_beta(slp, coef, alfa, mach, 'CL', 'c_l');
            subplot(2,3,5); plot_beta(slp, coef, alfa, mach, 'CM', 'c_m');
            subplot(2,3,6); plot_beta(slp, coef, alfa, mach, 'CN', 'c_n');
            
            if wind_axes
            subplot(2,3,1); plot_beta_data(slp, coef, 'cl');
            subplot(2,3,3); plot_beta_data(slp, coef, 'cd');
            else
            subplot(2,3,1); plot_beta_data(slp, coef, 'CX');
            subplot(2,3,3); plot_beta_data(slp, coef, 'CZ');
            end
            subplot(2,3,2); plot_beta_data(slp, coef, 'CY');
            subplot(2,3,4); plot_beta_data(slp, coef, 'CL');
            subplot(2,3,5); plot_beta_data(slp, coef, 'CM');
            subplot(2,3,6); plot_beta_data(slp, coef, 'CN');            
         end
    else
        for k=1:length(mach) 

            mch = mach(k);

            FigH = figure(k); clf
            set(FigH, 'NumberTitle', 'off', 'Name', sprintf('M=%0.2g⁰', mch));     

            if wind_axes
            subplot(2,3,1); plot_mach(mch, coef, alfa, beta, 'cl', 'c_L');
            subplot(2,3,3); plot_mach(mch, coef, alfa, beta, 'cd', 'c_D');
            else
            subplot(2,3,1); plot_mach(mch, coef, alfa, beta, 'CX', 'c_x');
            subplot(2,3,3); plot_mach(mch, coef, alfa, beta, 'CZ', 'c_z');
            end
            subplot(2,3,2); plot_mach(mch, coef, alfa, beta, 'CY', 'c_y');
            subplot(2,3,4); plot_mach(mch, coef, alfa, beta, 'CL', 'c_l');
            subplot(2,3,5); plot_mach(mch, coef, alfa, beta, 'CM', 'c_m');
            subplot(2,3,6); plot_mach(mch, coef, alfa, beta, 'CN', 'c_n');
            
            if wind_axes
            subplot(2,3,1); plot_mach_data(mch, coef, 'cl');
            subplot(2,3,3); plot_mach_data(mch, coef, 'cd');
            else            
            subplot(2,3,1); plot_mach_data(mch, coef, 'CX');
            subplot(2,3,3); plot_mach_data(mch, coef, 'CZ');
            end
            subplot(2,3,2); plot_mach_data(mch, coef, 'CY');
            subplot(2,3,4); plot_mach_data(mch, coef, 'CL');
            subplot(2,3,5); plot_mach_data(mch, coef, 'CM');
            subplot(2,3,6); plot_mach_data(mch, coef, 'CN');
        end
    end

end
% [mach x alfa x beta]
function plot_beta(slp, coef, alfa, mach, Fn, f)
    F = getfield(coef, Fn);
    r2d = 180/pi;
    
    C = interp3(coef.alpha, coef.mach, coef.beta, F, alfa, mach, slp, 'spline');  
    
    [A,B] = meshgrid(alfa*r2d, mach);
    mesh(A, B, C); zlabel(f); xlabel('α'); ylabel('M'); 
    a = min(min(C));
    b = max(max(C));
    if a == b
        a = -1; b = 1;
    end
    zlim([a b]);
    hold on;
    
    C = interp3(coef.alpha, coef.mach, coef.beta, F, coef.alpha, coef.mach, slp);  

    [N,M] = meshgrid(coef.alpha*r2d, coef.mach);
    plot3(N, M, C, '.'); 
    
    xlim([alfa(1) alfa(end)]*r2d);
    ylim([mach(1) mach(end)]);
    grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end
function plot_beta_data(slp, coef, Fn)
    F = getfield(coef, Fn);    
    r2d = 180/pi;
    [idx, frc] = find_index(coef.beta, slp);  
    C = squeeze(F(:,:,idx));

    [X,Y] = meshgrid(coef.alpha*r2d, coef.mach);
    plot3(X, Y, C, '-x'); 
    hold on;
    
    xlim([coef.alpha(1) coef.alpha(end)]*r2d);
    ylim([coef.mach(1) coef.mach(end)]);
    a = min(min(C));
    b = max(max(C));
    if a == b
        a = -1; b = 1;
    end
    zlim([a b]);

    grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end

% Vq = interp3(X,Y,Z,V,Xq,Yq,Zq)
% X=1:n, Y=1:m, Z=1:p, [n,m,p] where [m,n,p] = size(V)
function plot_mach(mch, coef, alfa, beta, Fn, f)
    F = getfield(coef, Fn);    

    r2d = 180/pi;
    [A,B] = meshgrid(beta, alfa);
    [idx, frc] = find_index(coef.mach, mch); % find_mach(coef, mch);
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
function plot_mach_data(mch, coef, Fn)
    r2d = 180/pi;
    [N,M] = meshgrid(coef.beta*r2d, coef.alpha*r2d);
    F = getfield(coef, Fn);    
    [idx, frc] = find_index(coef.mach, mch);
    C = squeeze(F(idx,:,:));
    plot3(M, N, C, '-x'); 
    hold on;
    
    xlim([coef.alpha(1) coef.alpha(end)]*r2d);
    ylim([coef.beta(1) coef.beta(end)]*r2d);
    zlim([min(min(C)) max(max(C))]);

    grid on; daspect 'manual'; rotate3d on; axis('vis3d');
end



