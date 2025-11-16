clc; clear;

% ==========================================================
% Sensor code 
% ==========================================================

% DLL name
DXL_LIB_NAME = 'dxl_x86_c';

if ~libisloaded(DXL_LIB_NAME)
    [notfound, warnings] = loadlibrary( ...
        DXL_LIB_NAME, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', ...
        'addheader', 'packet_handler.h');
end

% ---------------- Control Table Address (AX-12A, RAM Area) -----------
ADDR_TORQUE_ENABLE        = 24;   % 1 byte
ADDR_LED                  = 25;   % 1 byte
ADDR_GOAL_POSITION        = 30;   % 2 bytes
ADDR_MOVING_SPEED         = 32;   % 2 bytes
ADDR_TORQUE_LIMIT         = 34;   % 2 bytes
ADDR_PRESENT_POSITION     = 36;   % 2 bytes
ADDR_PRESENT_SPEED        = 38;   % 2 bytes
ADDR_PRESENT_LOAD         = 40;   % 2 bytes
ADDR_PRESENT_VOLTAGE      = 42;   % 1 byte
ADDR_PRESENT_TEMPERATURE  = 43;   % 1 byte
ADDR_MOVING               = 46;   % 1 byte

PROTOCOL_VERSION = 1.0;

% basic configuration motor/port
DXL_ID      = 1;                  % ID 
BAUDRATE    = 1000000;            % 1 Mbps (default of AX-12A) :contentReference[oaicite:2]{index=2}
DEVICENAME  = 'COM3';             % ex: 'COM3' at Windows, '/dev/ttyUSB0' at Linux

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

COMM_SUCCESS = 0;

%% ========================================================================

% PortHandler
port_num = portHandler(DEVICENAME);

% PacketHandler
packetHandler();

% Opens the port
if ~openPort(port_num)
    unloadlibrary(DXL_LIB_NAME);
    error('Falha ao abrir a porta %s.', DEVICENAME);
end

% Defines the baudrate
if ~setBaudRate(port_num, BAUDRATE)
    closePort(port_num);
    unloadlibrary(DXL_LIB_NAME);
    error('Falha ao configurar o baudrate %d.', BAUDRATE);
end

% Turns on the torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
else
    fprintf('AX-12A conectado e torque habilitado.\n');
end

%% ========================================================================
% LOG configuration
% ========================================================================

log_filename   = 'ax12a_log.csv';
num_samples    = 1000;    
sample_period  = 0.01;    

fid = fopen(log_filename, 'w');
if fid == -1
    error('Não foi possível abrir arquivo de log: %s', log_filename);
end

% head of CSV file (it will change)
fprintf(fid, ['time_s,' ...
              'goal_pos,moving_speed,torque_limit,' ...
              'present_pos,present_speed,present_load,' ...
              'present_voltage,present_temperature,' ...
              'torque_enable,led,moving\n']);

%% ========================================================================
% Reading loop
% ========================================================================

t0 = tic;

for k = 1:num_samples
    t = toc(t0);  

    % ---------- Reading 2 bytes (uint16) ----------
    goal_pos        = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION);
    moving_speed    = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED);
    torque_limit    = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_LIMIT);
    present_pos     = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    present_speed   = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_SPEED);
    present_load    = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_LOAD);
    
    % ---------- Reading 1 byte (uint8) ----------
    present_voltage     = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_VOLTAGE);
    present_temperature = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_TEMPERATURE);
    torque_enable_val   = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE);
    led_val             = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_LED);
    moving_val          = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING);
    
    % Checking error
    if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
    end
    
    % Writes in CSV File (It will change)
    fprintf(fid, '%.6f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n', ...
        t, ...
        goal_pos, moving_speed, torque_limit, ...
        present_pos, present_speed, present_load, ...
        present_voltage, present_temperature, ...
        torque_enable_val, led_val, moving_val);
    
    pause(sample_period);
end

fclose(fid);

% Turn off the torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);

closePort(port_num);
unloadlibrary(DXL_LIB_NAME);

fprintf('LOG finished. \n');
