% Ejemplo: dat= msdat_read('../../../DATCOM/T2-70'); show_datcom_deriv(dat)
function show_datcom_deriv(coef, mach, alfa, beta)

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
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CZA , alfa(a), mach, beta(b), 'spline');  
            subplot(4,2,1); plot(mach, c, '-'); grid on; ylabel('c_{z_a}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CZAD, alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,3); plot(mach, c, '-'); grid on; ylabel('c_{z_a_d}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CMA , alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,5); plot(mach, c, '-'); grid on; ylabel('c_{m_a}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CMAD, alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,7); plot(mach, c, '-'); grid on; ylabel('c_{m_a_d}'); hold on;

            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CYB , alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,2); plot(mach, c, '-'); grid on; ylabel('c_{y_b}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CLB , alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,4); plot(mach, c, '-'); grid on; ylabel('c_{l_b}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CNB , alfa(a), mach, beta(b), 'spline'); 
            subplot(4,2,6); plot(mach, c, '-'); grid on; ylabel('c_{n_b}'); hold on;

        end
        subplot(4,2,7); xlabel('M')
        subplot(4,2,6); xlabel('M')

        subplot(4,2,6);
        legend(lgnd,'Location','northeast');
    end

end

