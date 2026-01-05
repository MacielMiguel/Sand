function vmcController_XYZ()
%% ============================================
%  SAND Robot — Initial setup
% ============================================
clc; close all;

% Motors IDs
DXL_IDS = [1 2 3];  

% Registers addresses (AX-12, protocol 1.0)
ADDR_TORQUE_ENABLE    = 24;
ADDR_GOAL_POSITION    = 30;
ADDR_PRESENT_POSITION = 36;

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

% Communication setup
PROTOCOL_VERSION = 1.0;
BAUDRATE         = 1000000;     
DEVICENAME       = 'COM5';      

% Result codes
COMM_SUCCESS = 0;

% === Library selection ===
if strcmp(computer, 'PCWIN')
    LIB_NAME = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    LIB_NAME = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    LIB_NAME = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    LIB_NAME = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    LIB_NAME = 'libdxl_mac_c';
else
    error('Error: no recognition of OS.');
end

if ~libisloaded(LIB_NAME)
    fprintf('Charging library "%s"...\n', LIB_NAME);
    loadlibrary(LIB_NAME, 'dynamixel_sdk.h', ...
                'addheader','port_handler.h', ...
                'addheader','packet_handler.h');
end

% === Port handler setup ===
port_num = portHandler(DEVICENAME);
packetHandler();

% onCleanup: Ensure the port/library is reset even if an error occurs
cleanupObj = onCleanup(@() localCleanup(port_num, PROTOCOL_VERSION, ...
                                       DXL_IDS, ADDR_TORQUE_ENABLE, ...
                                       TORQUE_DISABLE, LIB_NAME));

% === Open serial port ===
fprintf('Open port %s...\n', DEVICENAME);
if openPort(port_num)
    fprintf('It was open succesfully!\n');
else
    error('Fail in openning port.');
end

% === Set baudrate ===
setBaudRate(port_num, BAUDRATE);

% === Torque activation for each motor ===
for id = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ...
                   ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error       = getLastRxPacketError(port_num, PROTOCOL_VERSION);

    if dxl_comm_result ~= COMM_SUCCESS
        error('[ID:%d] Error of communication: %s', ...
              id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        error('[ID:%d] Error of packet: %s', ...
              id, getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    fprintf(' -> Motor ID %d conected and torque activated.\n', id);
end

ADDR_MOVING_SPEED = 32;

speedVal = 100; % 100 slow, 400 medium, 700 fast
for id = DXL_IDS
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MOVING_SPEED, speedVal);
end

fprintf('\n SETUP FINISHED \n\n');

fprintf("Starting controller...\n");

try
    %% === Geometry (Moteur 1 = Latéral, Moteurs 2&3 = 6-Barres) ===
    L1 = 0.04;
    L2 = 0.10; 
    L3 = 0.10; 
    
    % Offset angles
    % theta1 = Lateral motor (Abduction)
    % theta2 = Motor thigh
    % theta3 = Motor knee
    theta1_offset = 0; 
    theta2_offset = 0; 
    theta3_offset = 0; 
    
    %% === VMC parameters ===
    % Kvmc remains aligned with the Cartesian axes [X, Y, Z]
    Kvmc = diag([0 60 50]); % Tuning parameters
    Dvmc = diag([0  4  4]);
    tauMax = [5.0; 5.0; 5.0]; 
    
    %% === Ellipse ===
    % Defined by the leg's workspace
    a = 0.03; 
    b = 0.015; 
    T = 4.0;
    omega = 2*pi/T;

    %% === Homing ===
    % Initialize the homing process for the motors
    q0 = [0; -2.206; -0.392]; % The ellipse will start from this point

    %  Wait until it reaches the point
    q_tol = deg2rad(2);   % tolerance
    t_max = 3.0;          % timeout 
    tH = tic;
   
    % Checks if it reached the point, if not, send it to there again
    while true
        for k = 1:numel(DXL_IDS)
            q_home = round(((q0(k) + deg2rad(150)) / deg2rad(300)) * 1023);
            q_home = max(0, min(1023, q_home));
    
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(k), ADDR_GOAL_POSITION, q_home);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end    
        end
        q_read = readSensors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_PRESENT_POSITION);
        q_corr = q_read(:) + [theta1_offset; theta2_offset; theta3_offset];
        if all(abs(q_corr - q0) < q_tol)
            break;
        end
        if toc(tH) > t_max
            fprintf('[Homing] Timeout, following anyway.\n');
            break;
        end
        fprintf("Pos atual = [%.3f %.3f %.3f]\n", q_corr);
        pause(0.02);
    end

    %% === First reading ===
    q_read = readSensors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_PRESENT_POSITION);     

    theta1 = q_read(1) + theta1_offset; 
    theta2 = q_read(2) + theta2_offset; 
    theta3 = q_read(3) + theta3_offset; 

    q = [theta1; theta2; theta3];
    
    % Initial FK 
    m = sqrt( (sin(theta3)^2) * (L3^2) + (cos(theta3)*L3 + L2)^2 );
    h = sqrt( L1^2 + m^2 );
    x = sin( pi/2 - atan(m/L1) - theta1 ) * h;
    y = cos( pi/2 - atan(m/L1) - theta1 ) * h;
    z = sin( pi - theta2 - atan( (sin(theta3)*L3) / (L2 + cos(theta3)*L3) ) ) * m;

    chi = [x; y; z];

    % Defining the ellipse starting point as follows
    chi_center = chi - [0; a; 0];
    chi0 = chi_center;
    
    fprintf("Initial pos = [%.3f %.3f %.3f]\n", chi);
    
    %% === State for derivatives ===
    q_prev  = q;
    dq_prev = [0;0;0];
    alpha_q = 1; 
    %t       = 0;
    
    %% === Live plot ===
    fig = figure('Name','6-Bar Leg – ZY','NumberTitle','off');
    hold on; grid on; axis equal;
    xlabel('Z [m]'); ylabel('Y [m]');
    title('Tracking ZY');

    tau_plot = linspace(0,T,400);
    chi_d_plot = zeros(3,numel(tau_plot));
    for k = 1:numel(tau_plot)
        chi_d_plot(:,k) = chi0 + [0; a*cos(omega*tau_plot(k)); b*sin(omega*tau_plot(k))];
    end
    plot(chi_d_plot(3,:), chi_d_plot(2,:), '--');   % ideal ellipse
    h_real = plot(chi(3), chi(2), 'o-', 'Color', [1 0.5 0]);
    legend('show');
        
    %% Before loop, starting the time
    t0     = tic;      % stopwatch to absolute time
    t_last = tic;      % stopwatch for measuring dt between iterations
    
    % (optional) frequency target, ex: 1000 Hz
    % dt_target = 0.001;
    
    %% === Principal loop ===
    while true
        % Real time
        t  = toc(t0);          % Absolute time since the start (seconds)
        dt_meas = toc(t_last); % dt real since the last iteration 
        t_last = tic;          % restart dt measurement for the next iteration
    
        % Protection: prevents division by zero/absurd values
        if dt_meas <= 0
            dt_meas = 1e-3;
        end
    
        % READING SENSORS
        q_read = readSensors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_PRESENT_POSITION);
    
        theta1 = q_read(1) + theta1_offset;  
        theta2 = q_read(2) + theta2_offset;  
        theta3 = q_read(3) + theta3_offset;  
    
        q = [theta1; theta2; theta3];
    
        % --- Velocity ---
        dq_raw  = (q - q_prev)/dt_meas;
        dq      = alpha_q*dq_raw + (1-alpha_q)*dq_prev;
        q_prev  = q;
        dq_prev = dq;
    
        % --- FK (Actual position) ---
        m = sqrt( (sin(theta3)^2) * (L3^2) + (cos(theta3)*L3 + L2)^2 );
        h = sqrt( L1^2 + m^2 );
        x = sin( pi/2 - atan(m/L1) - theta1 ) * h;
        y = cos( pi/2 - atan(m/L1) - theta1 ) * h;
        z = sin( pi - theta2 - atan( (sin(theta3)*L3) / (L2 + cos(theta3)*L3) ) ) * m;

        chi = [x; y; z];

        % --- Jacobian ---
        J = [ ...
            -L1*sin(theta1) - m*cos(theta1),  0,  (L2*L3*sin(theta3)*sin(theta1))/m; ...
             L1*cos(theta1) - m*sin(theta1),  0, -(L2*L3*sin(theta3)*cos(theta1))/m; ...
             0,  L2*cos(theta2) + L3*cos(theta2+theta3),  L3*cos(theta2+theta3) ...
        ];

        dchi = J * dq;
    
        % --- Desired trajectory ---
        chi_d  = chi0 + [0; a*cos(omega*t); b*sin(omega*t)];
        dchi_d =       [0; -a*omega*sin(omega*t); b*omega*cos(omega*t)];
    
        % --- Inputs VMC ---
        e  = chi_d  - chi;
        de = dchi_d - dchi;
    
        Fv = Kvmc*e + Dvmc*de;
    
        % --- Conversion Force ---
        tau = J.' * Fv;
        tau = max(min(tau, tauMax), -tauMax);
    
        % --- Writing to Motors ---
        writeMotors(port_num, PROTOCOL_VERSION, DXL_IDS, ...
                    ADDR_GOAL_POSITION, q_read, tau);
    
        % --- Update plot ---
        set(h_real, 'XData', [get(h_real,'XData') chi(3)], ...
                    'YData', [get(h_real,'YData') chi(2)]);
        drawnow("limitrate");
    
        % --- Debug ---
        fprintf("t=%.3f | dt=%.4f | LatTau=%.2f LegTau=[%.2f %.2f] | Y_err=%.3f\n", ...
                t, dt_meas, tau(1), tau(2), tau(3), e(2));
    
        % (Optional) If you want to try keeping ~dt_target
        slack = dt_target - dt_meas;
        if slack > 0, pause(slack); end
    end


catch ME
    fprintf('\n[vmcController] ERROR: %s\n', ME.message);
end

end 

% Subfunction to clean up the system in case of fail
function localCleanup(port_num, PROTOCOL_VERSION, DXL_IDS, ...
                      ADDR_TORQUE_ENABLE, TORQUE_DISABLE, LIB_NAME)

fprintf('\n[Cleanup] Releasing Dynamixel resources...\n');

% Desactivating the Torque
if ~isempty(DXL_IDS)
    for idc = DXL_IDS
        try
            write1ByteTxRx(port_num, PROTOCOL_VERSION, idc, ...
                           ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
        catch
            
        end
    end
end

% Closes port
try
    closePort(port_num);
catch
end

% clean library
try
    if libisloaded(LIB_NAME)
        unloadlibrary(LIB_NAME);
    end
catch
end

fprintf('[Cleanup] Done. Port and library released.\n');

end
