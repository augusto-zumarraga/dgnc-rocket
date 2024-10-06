function plot_ctrl_s1(flight, nfig, line_width, clr)

if nargin < 3 || isempty(line_width)
    line_width = 1;
end
if nargin < 4 || isempty(clr)
    clr = [1 0.5 0.25];
end
if nargin > 1 && ~isempty(nfig)
    f = figure(nfig); clf;
    f.Name = 'Control S1';
    f.NumberTitle = 'off';
end
gnc = flight.gnc;
dyn = flight.dyn;
frc = flight.frc;
flx = flight.flx;
rng = frc.on_off(1):frc.on_off(3);

t = gnc.t(rng);

clf


subplot(3,3,1);
r = flight.nav.eul(rng,1); 
p = flight.nav.eul(rng,2); 
y = flight.nav.eul(rng,3); 
plot(t, r, 'LineWidth', line_width, 'DisplayName', 'ϕ'); hold on
plot(t, p, 'LineWidth', line_width, 'DisplayName', 'θ'); 
plot(t, y, 'LineWidth', line_width, 'DisplayName', 'ψ'); 
grid on;

ylabel('[⁰]');
legend('show','AutoUpdate','off');

eul = to_euler(gnc.q_ref(rng,:))*180/pi;
plot(t, eul, '-.k');   

subplot(3,3,4); 
eul = to_euler(gnc.q_err(rng,:))*180/pi;
plot(t, eul, 'LineWidth', line_width);   
grid on;
legend('e_ϕ', 'e_θ', 'e_ψ'); 
ylabel('Euler error [⁰]');

subplot(3,3,7); 
plot(t, acosd(gnc.q_err(rng,1))*2, 'LineWidth', line_width); %, 'Color', clr); 
grid on;
ylabel('quaternion error angle [⁰]');

subplot(3,3,2); 
plot(t, gnc.w_err(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('angular rate error [⁰/s]');
legend('p', 'q', 'r');

subplot(3,3,5); 
plot(t, dyn.dy(rng), t, dyn.dz(rng), t, dyn.da(rng), t, dyn.rc(rng), 'LineWidth', line_width); 
ylim([-1.1 1.1])
grid on;
ylabel('actuators');
legend('d_y', 'd_z', 'd_a', 'rcs');

subplot(3,3,8); 
plot(t, flight.aer.alpha(rng), 'LineWidth', line_width, 'DisplayName', 'α'); 
hold on;
plot(t, flight.aer.beta (rng), 'LineWidth', line_width, 'DisplayName', 'β'); 
grid on;
ylim([-15 15]);
ylabel('[⁰]');
legend('show','AutoUpdate','off');


%%
subplot(3,3,3); 
plot(t, frc.fb(rng,:)/9.8067, 'LineWidth', line_width); 
grid on;
ylabel('fuerza especifica [g]');
legend('f_x', 'f_y', 'f_z');

subplot(3,3,6); 
plot(t, flx.y(rng,:), t, flx.z(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('η (deformacion)');
legend('y', 'z');

subplot(3,3,9); 
plot(t, flx.q(rng,:), t, flx.r(rng,:), 'LineWidth', line_width); 
grid on;
ylabel('dη/dt');
legend('y', 'z');


end

% roll, pitch, yaw  
function [eul] = to_euler(q)
    eul = zeros(length(q),3); 
    
    r = q(:,1);
    x = q(:,2);
    y = q(:,3);
    z = q(:,4);

    c31 =     2*(x.*z - r.*y);
    c11 = 1 - 2*(y.^2 + z.^2);
    c21 =     2*(x.*y + r.*z);
    c32 =     2*(y.*z + r.*x);
    c33 = 1 - 2*(x.^2 + y.^2);           

    eul(:,1) = atan2(c32, c33) ; % roll
    eul(:,2) = real(asin(-c31)); % pich
    eul(:,3) = atan2(c21, c11) ; % head
end

