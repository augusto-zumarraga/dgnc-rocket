% Ejemplo: dat= msdat_read('../../../DATCOM/T2-70'); show_datcom_deriv(dat)
function show_datcom_damp(coef, mach, alfa, beta)

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
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CYP, alfa(a), mach, beta(b), 'spline');  
            subplot(3,3,1); plot(mach, c, '-'); grid on; ylabel('c_{y_p}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CLP, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,4); plot(mach, c, '-'); grid on; ylabel('c_{l_p}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CNP, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,7); plot(mach, c, '-'); grid on; ylabel('c_{n_p}'); hold on;

            %%
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CXQ, alfa(a), mach, beta(b), 'spline');  
            subplot(3,3,2); plot(mach, c, '-'); grid on; ylabel('c_{x_q}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CZQ, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,5); plot(mach, c, '-'); grid on; ylabel('c_{z_q}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CMQ, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,8); plot(mach, c, '-'); grid on; ylabel('c_{m_q}'); hold on;

            %%
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CYR, alfa(a), mach, beta(b), 'spline');  
            subplot(3,3,3); plot(mach, c, '-'); grid on; ylabel('c_{y_r}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CLR, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,6); plot(mach, c, '-'); grid on; ylabel('c_{l_r}'); hold on;
            c = interp3(coef.alpha, coef.mach, coef.beta, coef.CNR, alfa(a), mach, beta(b), 'spline'); 
            subplot(3,3,9); plot(mach, c, '-'); grid on; ylabel('c_{n_r}'); hold on;

        end
        subplot(3,3,7); xlabel('M')
        subplot(3,3,8); xlabel('M')
        subplot(3,3,9); xlabel('M')

        subplot(3,3,9);
        legend(lgnd,'Location','northeast');
    end

end

