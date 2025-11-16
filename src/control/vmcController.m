function vmcController()
%% === Setup ===
TIME_STEP = wb_robot_get_basic_time_step();
dt = TIME_STEP / 1000;
wb_robot_init();
fprintf("Step 24: VMC (Front Left Leg) — continuous progress printing\n");

%% === Devices (Front Left Leg) ===
shoulderPitch_FL = wb_robot_get_device('shoulderPitch_FL');
knee_FL          = wb_robot_get_device('knee_FL');
shoulderPitchSensor_FL = wb_robot_get_device('shoulderPitch_FL_sensor');
kneeSensor_FL          = wb_robot_get_device('knee_FL_sensor');

wb_position_sensor_enable(shoulderPitchSensor_FL, TIME_STEP);
wb_position_sensor_enable(kneeSensor_FL, TIME_STEP);

wb_motor_set_position(shoulderPitch_FL, inf);
wb_motor_set_position(knee_FL, inf);
wb_motor_set_velocity(shoulderPitch_FL, 10);
wb_motor_set_velocity(knee_FL, 10);

%% === Parameters ===
L1 = 0.10; L2 = 0.10;
theta2_offset = -0.698;
theta3_offset =  1.390;

Kvmc = diag([10 10 20]);
Dvmc = diag([3 3 6]);
tauMax = [1.5; 1.5];

a = 0.015; b = 0.010; T = 60.0;
omega = 2*pi/T;

%% === Initial FK ===
wb_robot_step(TIME_STEP);
q2_raw = wb_position_sensor_get_value(shoulderPitchSensor_FL);
q3_raw = wb_position_sensor_get_value(kneeSensor_FL);
theta2 = q2_raw + theta2_offset;
theta3 = q3_raw + theta3_offset;
q = [theta2; theta3];

chi0 = [L1*cos(theta2)+L2*cos(theta2+theta3);
        -0.058;
        -L1*sin(theta2)-L2*sin(theta2+theta3)];
fprintf("Anchored ellipse center (FK-based) at chi0=[%.3f, %.3f, %.3f]\n", chi0);
fprintf("Initial model angles: theta2=%.3f, theta3=%.3f\n\n", theta2, theta3);

q_prev=q; dq_prev=[0;0]; alpha_q=0.2; t=0;

%% === Live plot ===
fig = figure('Name','χx×χz – FL leg','NumberTitle','off');
hold on; grid on; axis equal;
xlabel('χx [m]'); ylabel('χz [m]');
title('Trajectory (Front Left Leg)');
tau_plot=linspace(0,T,400);
chi_d_plot=zeros(3,numel(tau_plot));
for k=1:numel(tau_plot)
    chi_d_plot(:,k)=chi0+[a*cos(omega*tau_plot(k));0;b*sin(omega*tau_plot(k))];
end
plot(chi_d_plot(1,:),chi_d_plot(3,:),'--');
h_real=plot(nan,nan,'o-','Color',[1 0.5 0]);
legend('χ_d (desired)','χ (real)');

%% === Main loop ===
while wb_robot_step(TIME_STEP) ~= -1
    t = t + dt;

    % Read sensors
    q2_raw = wb_position_sensor_get_value(shoulderPitchSensor_FL);
    q3_raw = wb_position_sensor_get_value(kneeSensor_FL);
    theta2 = q2_raw + theta2_offset;
    theta3 = q3_raw + theta3_offset;
    q = [theta2; theta3];

    dq_raw = (q - q_prev)/dt;
    dq = alpha_q*dq_raw + (1-alpha_q)*dq_prev;
    q_prev=q; dq_prev=dq;

    % FK and Jacobian
    chi = [L1*cos(theta2)+L2*cos(theta2+theta3);
           -0.058;
           -L1*sin(theta2)-L2*sin(theta2+theta3)];
    J = [-L1*sin(theta2)-L2*sin(theta2+theta3), -L2*sin(theta2+theta3);
          0, 0;
         -L1*cos(theta2)-L2*cos(theta2+theta3), -L2*cos(theta2+theta3)];
    dchi = J*dq;

    % Desired motion
    chi_d = chi0 + [a*cos(omega*t); 0; b*sin(omega*t)];
    dchi_d = [-a*omega*sin(omega*t); 0; b*omega*cos(omega*t)];

    % VMC
    e = chi - chi_d; de = dchi - dchi_d;
    Fv = -Kvmc*e - Dvmc*de;
    tau = J.' * Fv;
    tau = max(min(tau, tauMax), -tauMax);

    % Apply torque
    wb_motor_set_torque(shoulderPitch_FL, tau(1));
    wb_motor_set_torque(knee_FL, tau(2));

    % === Print progress every step ===
    fprintf("t=%.2f | tau=[%.3f %.3f] | q=[%.3f %.3f] | chi=[%.3f %.3f %.3f] | chi_d=[%.3f %.3f %.3f]\n", ...
        t, tau(1), tau(2), q(1), q(2), chi(1), chi(2), chi(3), chi_d(1), chi_d(2), chi_d(3));
    drawnow("limitrate");
end

wb_robot_cleanup();
end