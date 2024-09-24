% Ejemplo: dat= msdat_read('../../../DATCOM/T2-70'); show_datcom_coefs(dat)
function show_datcom_coefs(coef, mach, alfa, beta)

    if nargin < 2 || isempty(mach)
        mach = [0:0.1:0.6 0.7:0.025:1.5 1.6:0.1:5];
    end 
    if nargin < 3 || isempty(alfa)
        alfa = [0:1:5 7 9 12] * pi/180;
    end 
    if nargin < 4
        beta = [0:2:10  ] * pi/180;
    else
        beta = beta * pi/180;
    end 

    lgnd = cell(1,length(alfa));
    
    c = zeros(size(mach));
    for b=1:length(beta) 
    
        FigH = figure(b); 
        clf
        set(FigH, 'NumberTitle', 'off', 'Name', sprintf('β=%0.2g⁰', beta(b)*180/pi));        
        
        for a=1:length(alfa) 

            lgnd{a} = sprintf('α=%0.2g⁰', alfa(a)*180/pi);

            %%
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CX , alfa(a), mach, beta(b), 'spline');  
            subplot(3,2,1); plot(mach, c, '-'); grid on; ylabel('c_x'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CY , alfa(a), mach, beta(b), 'spline'); 
            subplot(3,2,3); plot(mach, c, '-'); grid on; ylabel('c_y'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CZ , alfa(a), mach, beta(b), 'spline'); 
            subplot(3,2,5); plot(mach, c, '-'); grid on; ylabel('c_z'); hold on;

            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CL , alfa(a), mach, beta(b), 'spline'); 
            subplot(3,2,2); plot(mach, c, '-'); grid on; ylabel('c_l'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CM , alfa(a), mach, beta(b), 'spline'); 
            subplot(3,2,4); plot(mach, c, '-'); grid on; ylabel('c_m'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CN , alfa(a), mach, beta(b), 'spline'); 
            subplot(3,2,6); plot(mach, c, '-'); grid on; ylabel('c_n'); hold on;

        end
        subplot(3,2,5); xlabel('M')
        subplot(3,2,6); xlabel('M')

        subplot(3,2,6);
        legend(lgnd,'Location','northeast');
    end

end

