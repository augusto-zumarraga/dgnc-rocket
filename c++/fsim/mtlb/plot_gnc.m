function plot_gnc(flight, line_width, clr, nfig, rng)

if nargin < 2 || isempty(line_width)
    line_width = 1;
end
if nargin < 3 || isempty(clr)
    clr = [1 0.5 0.25];
end
if nargin > 3 || isempty(nfig)
    f = figure(nfig); clf;
    f.Name = 'Guidance';
    f.NumberTitle = 'off';
end
gnc = flight.gnc;
if nargin < 4 || isempty(rng)
    rng = find(flight.frc > 0, 1, 'first'):find(flight.frc > 0, 1, 'last');    
end
t = gnc.t(rng);
m2k = 1e-3;
clf
subplot(3,2,1); 
plot(t, gnc.r_dir(rng,:), 'LineWidth', line_width); %, 'Color', clr); 
legend('x^i', 'y^i', 'z^i');
legend('autoupdate', 'off');
hold on;
plot(t, gnc.x_dir(rng), '-.k');
grid on;
ylabel('pointing');

subplot(3,2,3); 
plot(t, gnc.e_nrm(rng), 'LineWidth', line_width); %, 'Color', clr); 
ylim([0 10]);
grid on;
ylabel('pointing error norm [⁰]');

subplot(3,2,5); 
plot(t, gnc.g_st(rng), 'LineWidth', line_width); 
grid on;
ylabel('Estado de la navegación');

%%
subplot(3,2,2); 
plot(t, gnc.ttgo(rng), 'LineWidth', line_width); 
grid on;
ylabel('Time To Go');

subplot(3,2,4); 
plot(t, gnc.rtgo(rng)*m2k, 'LineWidth', line_width); 
grid on;
ylabel('Range To Go [km]');

subplot(3,2,6); 
plot(t, gnc.rbis(rng)*m2k, 'LineWidth', line_width); 
grid on;
ylabel('Rbias [km]');

