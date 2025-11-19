function matlab_controller()

TIME_STEP = wb_robot_get_basic_time_step();
dt        = TIME_STEP / 1000;
wb_robot_init();
fprintf("Starting controller...\n");

%% === Reference of the devices ===
shoulderLat_FR       = wb_robot_get_device('shoulderLat_FR');       % θ1
shoulderPitch_FR = wb_robot_get_device('shoulderPitch_FR');     % θ2
knee_FR          = wb_robot_get_device('knee_FR');              % θ3

shoulderLat_FR_sensor       = wb_robot_get_device('shoulderLat_FR_sensor');
shoulderPitchSensor_FR = wb_robot_get_device('shoulderPitch_FR_sensor');
kneeSensor_FR          = wb_robot_get_device('knee_FR_sensor');

wb_position_sensor_enable(shoulderLat_FR_sensor,       TIME_STEP);
wb_position_sensor_enable(shoulderPitchSensor_FR, TIME_STEP);
wb_position_sensor_enable(kneeSensor_FR,          TIME_STEP);

% Motors in torque mode
wb_motor_set_position(shoulderLat_FR,       inf);
wb_motor_set_position(shoulderPitch_FR, inf);
wb_motor_set_position(knee_FR,          inf);

wb_motor_set_velocity(shoulderLat_FR,       0);
wb_motor_set_velocity(shoulderPitch_FR, 0);
wb_motor_set_velocity(knee_FR,          0);

%% === Geometry ===
L  = 0.30; % Longueur du chassis central
W  = 0.115; % Largeur du chassis central
Hb = 0.06; % Hauteur du chassis central

l1 = 0.06;
l2 = 0.10;
l3 = 0.10;

% Angles offset
theta1_offset = 0.0;
theta2_offset = -0.698;
theta3_offset =  1.390;

%% === VMC parameters ===
Kvmc = diag([60 0 80]);
Dvmc = diag([4  0  5]);
% --- Gains intégrateurs (I) ---
Kivmc = diag([100 0 100]);        % commence petit ! (axe x et z uniquement)
tauMax = [3; 3; 3];

%% === Ellipse ===
a = 0.010; 
b = 0.006;  
T = 6.0;
omega = 2*pi/T;

%% === First reading and FK to find chi0 ===
wb_robot_step(TIME_STEP);

q1_raw = wb_position_sensor_get_value(shoulderLat_FR_sensor);
q2_raw = wb_position_sensor_get_value(shoulderPitchSensor_FR);
q3_raw = wb_position_sensor_get_value(kneeSensor_FR);

theta1 = q1_raw + theta1_offset;
theta2 = q2_raw + theta2_offset;
theta3 = q3_raw + theta3_offset;

q = [theta1; theta2; theta3];

s1  = sin(theta1);  c1  = cos(theta1);
s2  = sin(theta2);  c2  = cos(theta2);
s23 = sin(theta2 + theta3);
c23 = cos(theta2 + theta3);

chi = [
    L/2   +  l2*s2              +  l3*s23;
    -W/2  +  l1*s1 + l2*s1*c2   +  l3*s1*c23;
    -Hb/2 -  l1*c1 - l2*c1*c2   -  l3*c1*c23
];

% Ellipse center: start the trajectory at where is the foot in t = 0
chi_center = chi - [a; 0; 0];
chi0 = chi_center;

fprintf("Initial χ = [%.3f %.3f %.3f]\n", chi);
fprintf("Ellipse center = [%.3f %.3f %.3f]\n", chi_center);
fprintf("Joint angles (theta) = [%.3f %.3f %.3f]\n\n", theta1, theta2, theta3);

%% === State for derivatives ===
q_prev  = q;
dq_prev = [0;0;0];
alpha_q = 0.2;   % exponential filter
% Etat intégral sur l’erreur en espace cartésien
e_int   = [0;0;0];            % intégrale de e = chi_d - chi

% Limites anti-windup sur l’intégrale
e_int_max = [0.01; 0.0; 0.01];    % à adapter, mais petit au début
t       = 0;

%% === Live plot χx vs χz ===
fig = figure('Name','RF leg – χx×χz','NumberTitle','off');
hold on; grid on; axis equal;
xlabel('χ_x [m]'); ylabel('χ_z [m]');
title('Tracking on XZ');

tau_plot = linspace(0,T,400);
chi_d_plot = zeros(3,numel(tau_plot));
for k = 1:numel(tau_plot)
    chi_d_plot(:,k) = chi0 + [a*cos(omega*tau_plot(k)); 0; b*sin(omega*tau_plot(k))];
end
plot(chi_d_plot(1,:), chi_d_plot(3,:), '--');   % ideal ellipse
h_real = plot(chi(1), chi(3), 'o-', 'Color', [1 0.5 0]);

%% === Principal loop ===
while wb_robot_step(TIME_STEP) ~= -1

    % --- Sensors ---
    q1_raw = wb_position_sensor_get_value(shoulderLat_FR_sensor);
    q2_raw = wb_position_sensor_get_value(shoulderPitchSensor_FR);
    q3_raw = wb_position_sensor_get_value(kneeSensor_FR);

    theta1 = q1_raw + theta1_offset;
    theta2 = q2_raw + theta2_offset;
    theta3 = q3_raw + theta3_offset;

    q = [theta1; theta2; theta3];


    % --- Velocity ---
    dq_raw  = (q - q_prev)/dt;
    dq      = alpha_q*dq_raw + (1-alpha_q)*dq_prev;
    q_prev  = q;
    dq_prev = dq;

    % --- FK to obtain [x,y,z] for end affector ---
    s1  = sin(theta1);  c1  = cos(theta1);
    s2  = sin(theta2);  c2  = cos(theta2);
    s23 = sin(theta2 + theta3);
    c23 = cos(theta2 + theta3);

    chi = [
        L/2   +  l2*s2              +  l3*s23;
        -W/2  +  l1*s1 + l2*s1*c2   +  l3*s1*c23;
        -Hb/2 -  l1*c1 - l2*c1*c2   -  l3*c1*c23
    ];

    % --- Jacobian for FR leg ---
    J = [
        0,                             l2*c2 + l3*c23,                   l3*c23;
        l1*c1 + l2*c1*c2 + l3*c1*c23, -l2*s1*s2 - l3*s1*s23,            -l3*s1*s23;
        l1*s1 + l2*s1*c2 + l3*s1*c23,  l2*c1*s2 + l3*c1*s23,             l3*c1*s23
    ];

    dchi = J * dq;

    % --- Desired trajectory ---
    chi_d  = chi0   + [a*cos(omega*t);       0; b*sin(omega*t)];
    dchi_d =          [-a*omega*sin(omega*t); 0; b*omega*cos(omega*t)];

    % --- Pre-Stabilization
    % if t < t_start_traj
    %     chi_d  = chi;             % on fige la consigne à la position actuelle
    %     dchi_d = [0;0;0];         % on demande vitesse nulle
    % else
    %     tau_t = t - t_start_traj; % horloge interne de la trajectoire
    %     chi_d  = chi0 + [a*cos(omega*tau_t); 0; b*sin(omega*tau_t)];
    %     dchi_d = [ -a*omega*sin(omega*tau_t); 0; b*omega*cos(omega*tau_t)];
    % end

    % --- Inputs VMC ---
    e   = chi_d  - chi;
    de  = dchi_d - dchi;
    % === Intégrateur sur e (χ) ===
    e_int = e_int + e * dt;       % intégration simple
    % Anti-windup : on sature l’état intégral
    e_int = min(max(e_int, -e_int_max), e_int_max);
    Fv  = Kvmc*e + Dvmc*de + Kivmc*e_int;

    tau = J.' * Fv;
    tau = max(min(tau, tauMax), -tauMax);

    % --- Apply torques ---
    wb_motor_set_torque(shoulderLat_FR,       tau(1));
    wb_motor_set_torque(shoulderPitch_FR, tau(2));
    wb_motor_set_torque(knee_FR,          tau(3));

    % --- Update plot ---
    set(h_real, 'XData', [get(h_real,'XData') chi(1)], ...
                'YData', [get(h_real,'YData') chi(3)]);
    drawnow("limitrate");

    % --- Debug ---
    fprintf("t=%.2f | au=[%.3f %.3f %.3f] | q=[%.3f %.3f %.3f]\n", ...
            t, tau(1), tau(2), tau(3), q(1), q(2), q(3));
    fprintf("       | chi=[%.3f %.3f %.3f] | chi_d=[%.3f %.3f %.3f]\n", ...
            chi(1), chi(2), chi(3), chi_d(1), chi_d(2), chi_d(3));
    fprintf("       | dchi=[%.3f %.3f %.3f] | dchi_d=[%.3f %.3f %.3f] Fv=[%.3f %.3f %.3f]\n\n", ...
            dchi(1), dchi(2), dchi(3), dchi_d(1), dchi_d(2), dchi_d(3), Fv(1), Fv(2), Fv(3));

    t = t + dt;
end

wb_robot_cleanup();
end
