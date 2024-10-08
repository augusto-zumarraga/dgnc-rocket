function plot_tray(flight, line_width, param, clr, nfig, nplot)

if nargin < 2 || isempty(line_width)
    line_width = 1;
end
if nargin < 3
    param = [];
end
if nargin < 4 || isempty(clr)
    clr = [0 0 0];
end
if nargin > 4 && ~isempty(nfig)
    f = figure(nfig); clf;
    f.Name = 'Flight';
    f.NumberTitle = 'off';
else
    clf;
end
if nargin < 6
    nplot = 0;
end
% https://www.argentina.gob.ar/ciencia/conae/centros-y-estaciones/estacion-terrena-cordoba/ubicacion
CETT = [-31.524075, -64.463522, 730];


go  = 9.80665;
nav = flight.nav;
aer = flight.aer;
frc = flight.frc;

rng = find(flight.t < 500);
rng = 1:rng(end);
t = flight.t(rng);


% posicion ECEF de la antena
p_cett = lla2ecef(CETT)*1e-3;
% vertical geocentrica del lugar
v_cett = p_cett/sqrt(sum(p_cett.*p_cett)); 
% posicion relativa de la antena
d_cett = [nav.pos(:,1)-p_cett(1), nav.pos(:,2)-p_cett(2) , nav.pos(:,3)-p_cett(3)];
% distancia a la antena
r_cett = sqrt( d_cett(:,1).*d_cett(:,1) ...
             + d_cett(:,2).*d_cett(:,2) ...
             + d_cett(:,3).*d_cett(:,3));
% elevacion de la antena
sn = sind(1); 
% altura sobre el umbar de deteccion
h_cett = d_cett(:,1)*v_cett(1) ...
       + d_cett(:,2)*v_cett(2) ...
       + d_cett(:,3)*v_cett(3) - r_cett * sn;


%% ------------------------------------------------------------------------
%                                                                       NAV
if ~nplot
    subplot(4,3,1); 
end
if ~nplot || nplot == 1
    plot(flight.t, nav.h, 'LineWidth', line_width, 'Color', clr); 
    hold on;
    plot(flight.t, h_cett, ':k'); 
    grid on;
    ylabel('h [km]');
    legend('h_{orb}', 'h_{cett}');
end

if ~nplot
    subplot(4,3,4); 
end
if ~nplot || nplot == 4
    plot(flight.t, nav.r, 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('r [km]');
end

if ~nplot
    subplot(4,3,7); 
end
if ~nplot || nplot == 7
    plot(flight.t, nav.v, 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('v [m/s]');
end

if ~nplot
    subplot(4,3,10); 
end
if ~nplot || nplot == 10
    if isfield(nav, 'eul')
        p = nav.eul(rng,2); 
    else
        p = nav.p(rng); 
    end
    if isfield(nav, 'gamma')
        y = nav.gamma(rng);
        k = find(nav.v(rng) > 0,1); 
        y(1:k) = p(1:k);
        plot(t, y, 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'γ');
        hold on;
    end
    plot(t, p, '--', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'θ'); 
    grid on;
    % if ~isempty(param) && isfield(param, 'q_ref')
    %     eul = quaternion.to_euler(param.q_ref);
    %     t   = (0:length(eul)-1)*param.ts;
    %     plot(t, eul(:,2));
    %     xlim([0 max(flight.t)]);
    % end
    ylabel('[⁰]');
    legend('show','AutoUpdate','off');
end

%% ------------------------------------------------------------------------
%                                                                     AERO
if isfield(aer, 'alpha')
    a = aer.alpha(rng); 
    lbl = 'α_a';
else
    a = nav.p(rng)-nav.y(rng); 
    lbl = 'α=θ-γ';
end

if ~nplot
    subplot(4,3,2); 
end
if ~nplot || nplot == 2
    plot(t, aer.M(rng), 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('Mach');
    xlabel('t [s]')
end

if ~nplot
    subplot(4,3,5); 
end
if ~nplot || nplot == 5
    plot(t, aer.Q(rng), 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('Q [Pa]');
    
    yyaxis right
    plot(t, aer.Q(rng).*a, '-.', 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('Q.α [Pa.⁰]');
    yyaxis left
end

if ~nplot
    subplot(4,3,8);
end
if ~nplot || nplot == 8
    plot(t, a, 'LineWidth', line_width, 'Color', clr, 'DisplayName', lbl); 
    hold on;
    grid on;
    if isfield(aer, 'beta')
        plot(t, aer.beta(rng), '--', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'β_a'); 
    end
    if isfield(nav, 'alpha')
        plot(t, nav.alpha(rng), '-.', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'α_T'); 
    end
    if isfield(nav, 'beta')
        plot(t, nav.beta(rng), ':', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'β_T'); 
    end
    ylabel('[⁰]');
    legend('show','AutoUpdate','off');
end

if ~nplot
    subplot(4,3,11); 
end
if ~nplot || nplot == 11
    plot(t, frc.T(rng)/go, 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'thrust'); 
    hold on;
    if isfield(nav, 'Fa')
    plot(t, aer.Fa(rng,1)/go, '--', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'aero axial'); 
    plot(t, aer.Fa(rng,2)/go, '-.', 'LineWidth', line_width, 'Color', clr, 'DisplayName', 'aero normal'); 
    end
    grid on;
    ylabel('F [kgf]');
    xlabel('t [s]')
    legend('show','AutoUpdate','off');
end

%% ------------------------------------------------------------------------ 
%                                                                      ACT
if ~nplot
    subplot(4,3,3); 
end
if ~nplot || nplot == 3
    if isfield(nav, 'eul')
        r = nav.eul(rng,1); 
        p = nav.eul(rng,2); 
        y = nav.eul(rng,3); 
        plot(t, r, 'LineWidth', line_width, 'DisplayName', 'ϕ'); 
        hold on
        plot(t, p, 'LineWidth', line_width, 'DisplayName', 'θ'); 
        plot(t, y, 'LineWidth', line_width, 'DisplayName', 'ψ'); 
        grid on;
    
        ylabel('[⁰]');
        legend('show','AutoUpdate','off');
    end
end

%% ------------------------------------------------------------------------ 
%                                                                      LOSS
% if isfield(flight, 'loss')
%     subplot(2,3,3); 
%     hold on;
% 
%     line = {'k' 'b' 'g' 'r' 'c' 'y' };
% 
%     for k=1:length(flight.loss)
%         plot(flight.t, flight.loss(k).data, line{k}, 'LineWidth', line_width, 'DisplayName', flight.loss(k).name); 
%     end
%     grid on;
%     legend('show','AutoUpdate','off');
%     plot(0, 0); 
% end

if ~nplot
    subplot(4,3,6);
end
if ~nplot || nplot == 6
    plot(t, flight.m(rng), 'LineWidth', line_width, 'Color', clr); 
    hold on;
    grid on;
    ylabel('m [kg]');
    xlabel('t [s]')
end

if ~nplot
    subplot(2,3,6);
end
if ~nplot || nplot == -1
    geoplot(nav.lat,nav.lng, '.', 'MarkerSize', 3);
end




