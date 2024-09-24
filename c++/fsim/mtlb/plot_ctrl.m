function plot_ctrl(flight, line_width, clr, nfig, rng)

if nargin < 2 || isempty(line_width)
    line_width = 1;
end
if nargin < 3 || isempty(clr)
    clr = [1 0.5 0.25];
end
if nargin > 3 
    f = figure(nfig); clf;
    f.Name = 'Control';
    f.NumberTitle = 'off';
end

gnc = flight.gnc;
dyn = flight.dyn;
frc = flight.frc;
flx = flight.flx;
if nargin < 4 || isempty(rng)
    rng = find(flight.frc > 0, 1, 'first'):find(flight.frc > 0, 1, 'last');    
end
t = gnc.t(rng);

clf
subplot(3,3,1); 
plot(t, gnc.r_dir(rng,:), 'LineWidth', line_width); %, 'Color', clr); 
legend('x^i', 'y^i', 'z^i');
legend('autoupdate', 'off');
hold on;
plot(t, gnc.x_dir(rng,:), '-.k');
grid on;
ylabel('pointing');

subplot(3,3,4); 
plot(t, atan2d(gnc.e_dir(rng,2:3), gnc.e_dir(rng,1)), 'LineWidth', line_width); %, 'Color', clr); 
grid on;
legend('y^b', 'z^b');
ylabel('pointing error [⁰]');
ylim([-1 1])

subplot(3,3,7); 
plot(t, gnc.e_nrm(rng), 'LineWidth', line_width); %, 'Color', clr); 
grid on;
ylabel('pointing error norm');

subplot(3,3,2); 
plot(t, gnc.w_err(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('angular rate error [⁰/s]');
legend('p', 'q', 'r');

subplot(3,3,5); 
plot(t, dyn.dy(rng), t, dyn.dz(rng), t, dyn.da(rng), t, dyn.rc(rng), 'LineWidth', line_width); 
grid on;
ylabel('actuators');
legend('d_y', 'd_z', 'd_a', 'rcs');

subplot(3,3,8); 
plot(t, frc.fb(rng,:)/9.8067, 'LineWidth', line_width); 
grid on;
ylabel('fuerza especifica [g]');
legend('f_x', 'f_y', 'f_z');

%%
subplot(3,3,3); 
plot(t, flx.y(rng,:), t, flx.z(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('η (deformacion)');
legend('y', 'z');

subplot(3,3,6); 
plot(t, flx.q(rng,:), t, flx.r(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('dη/dt');
legend('y', 'z');

subplot(3,3,9);
r = flight.nav.eul(rng,1); 
p = flight.nav.eul(rng,2); 
y = flight.nav.eul(rng,3); 
plot(t, r*180/pi, 'LineWidth', line_width, 'DisplayName', 'ϕ'); hold on
plot(t, p*180/pi, 'LineWidth', line_width, 'DisplayName', 'θ'); 
plot(t, y*180/pi, 'LineWidth', line_width, 'DisplayName', 'ψ'); 
grid on;

ylabel('[⁰]');
legend('show','AutoUpdate','off');

