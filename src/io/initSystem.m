function hw = initSystem(config)
% Initializes the hardware or simulation interface
    lib_name = '';
    
    if strcmp(computer, 'PCWIN')
      lib_name = 'dxl_x86_c';
    elseif strcmp(computer, 'PCWIN64')
      lib_name = 'dxl_x64_c';
    elseif strcmp(computer, 'GLNX86')
      lib_name = 'libdxl_x86_c';
    elseif strcmp(computer, 'GLNXA64')
      lib_name = 'libdxl_x64_c';
    elseif strcmp(computer, 'MACI64')
      lib_name = 'libdxl_mac_c';
    end
    
    % Load Libraries
    if ~libisloaded(lib_name)
        [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    end

    hw.type = config.hw.type;

    switch hw.type
        case "dynamixel"
            % Control table address
            ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
            ADDR_MX_GOAL_POSITION       = 30;
            ADDR_MX_PRESENT_POSITION    = 36;

            % Protocol version
            PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel
            
            % Default setting
            DXL_ID                      = 1;            % Dynamixel ID: 1
            BAUDRATE                    = 1000000;
            DEVICENAME                  = 'COM5';       % Check which port is being used on your controller
                                                        % ex) Windows: 'COM5'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
            
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            TORQUE_DISABLE              = 0;            % Value for disabling the torque
            DXL_MINIMUM_POSITION_VALUE  = 100;          % Dynamixel will rotate between this value
            DXL_MAXIMUM_POSITION_VALUE  = 4000;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
            DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold
            
            ESC_CHARACTER               = 'e';          % Key for escaping loop
            
            COMM_SUCCESS                = 0;            % Communication Success result value
            COMM_TX_FAIL                = -1001;        % Communication Tx Failed

            % Initialize PortHandler Structs
            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            port_num = portHandler(DEVICENAME);
            
            % Initialize PacketHandler Structs
            packetHandler();
            
            index = 1;
            dxl_comm_result = COMM_TX_FAIL;             % Communication result
            dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
            
            dxl_error = 0;                              % Dynamixel error
            dxl_present_position = 0;                   % Present position
           
            % Open port
            if (openPort(port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(lib_name);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end
            
            % Set port baudrate
            if (setBaudRate(port_num, BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

            fprintf('Initializing hardware at port %s...\n', port_num);

        case "sim"
            % Simulating in Webots
            fprintf('Starting simulate mode\n');
    end
end
