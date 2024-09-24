%
% ts   : tiempo de muestreo para la tabla
% ti   : tiempo de inicio de la transición (sin suavizar)
% te   : tiempo final de la tabla
% p1   : ángulo(s) inicial(es) - si son 3 se computa el cuaternión
% p2   : ángulo(s) final(es)
% q_max: máxima velocidad de cabeceo
% t_flt: tiempo para la suavización
% nfig : número de figura si queremos graficar los resultados
% 
% p    : resultados (si p1 y p2 tienen tres componentes, el resultado son
%        las cuatro componentes del cuaternión
%
% ejemplo:
%
%     d2r = pi/180; q = pitch_maneuver(0.01, 15, 40, [0 90 155]*d2r, [0 87 155]*d2r, 1*d2r, 4, 1);
%
function p = pitch_maneuver(ts, ti, te, p1, p2, q_max, t_flt, nfig)
   
    if nargin < 8 
        nfig = 0;
    end

    dp = p2 - p1;
    dt = floor(abs(dp)/q_max/ts)*ts;
    dt = max(dt);
    p  = zeros(length(p1), floor(te/ts)+1);
    u  = p;
    for k=1:length(p1) 
        if p1(k) == p2(k)
            p(k,:) = ones(1,length(p)) * p1(k);
            u(k,:) = p(k,:);
        else
            [p(k,:), u(k,:)] = build(ts, ti, te, p1(k), p2(k), dt, dp(k), t_flt);
        end
    end
    
    r2d = 180/pi;
    if length(p1) == 3 
        p = quaternion.from_euler(p(1,:), p(2,:), p(3,:));
        if nfig > 0
            e = quaternion.to_euler(p);
            figure(nfig);
            clf
            for k=1:3
                subplot(4,1,k);
                hold on;
                t = (0:length(p)-1)*ts;
                plot(t , u(k,:)*r2d, '.');
                plot(t , e(:,k)*r2d, 'LineWidth', 2);
                grid on;
            end
            subplot(414);
            plot(t , p, 'LineWidth', 2);
            legend('r', 'x', 'y', 'z');
        end
    else
        if nfig > 0
            figure(nfig);
            clf
            for k=1:length(p1)
                subplot(length(p1),1,k);
                hold on;
                t = (0:length(p)-1)*ts;
                plot(t , u(k,:), '.');
                plot(t , p(k,:), 'LineWidth', 2);
                grid on;
            end
        end
    end
    
end

function [p, u] = build(ts, ti, te, p1, p2, dt, dp, t_flt)
    t1 = 0:ts:ti-ts;
    u1 = ones(size(t1)) * p1;

    t2 = 0:ts:dt-ts;
    u2 = p1 + t2*dp/dt;
    t2 = t2 + ti;

    t3 = t2(end)+ts:ts:te;
    u3 = ones(size(t3)) * p2;

    if isempty(u1)
        error 'tiempo insuficiente para iniciar la maniobra';
    end
    if isempty(u3)
        error 'tiempo insuficiente para ejecutar la maniobra';
    end

    u = [u1 u2 u3]';
    if t_flt > 0
        p = suavizar(u, floor(t_flt/ts));
    else
        p = u;
    end
end