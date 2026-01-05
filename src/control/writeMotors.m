function writeMotors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_GOAL_POSITION, q_initial, tau)              
    % writeMotors
    % -------------------------------------------------------------------------
    % Sends goal position commands to multiple Dynamixel motors based on
    % virtual torques and the current joint positions.
    %
    % This function implements a simple torque-to-position mapping, where
    % virtual joint torques (typically computed by a VMC controller) are
    % converted into angular increments. The resulting joint positions are
    % saturated within mechanical limits, mapped to encoder units, and written
    % to the motors using the Dynamixel SDK.
    %
    % -------------------------------------------------------------------------
    % Syntax:
    %   writeMotors(port_num, PROTOCOL_VERSION, DXL_IDS, ...
    %               ADDR_GOAL_POSITION, q_initial, tau)
    %
    % -------------------------------------------------------------------------
    % Inputs:
    %   port_num : Port handler returned by portHandler() after opening
    %              the serial communication.
    %
    %   PROTOCOL_VERSION : Dynamixel communication protocol version
    %                      (e.g., 1.0 for AX-series, 2.0 for X-series).
    %
    %   DXL_IDS  : Row or column vector containing the IDs of the motors
    %              to be commanded (e.g., [1 2 3]).
    %
    %   ADDR_GOAL_POSITION : Address of the "Goal Position" register in the
    %                        Dynamixel control table
    %                        (e.g., 30 for AX-12A, Protocol 1.0).
    %
    %   q_initial : Column vector (nMotors x 1) containing the current joint
    %               positions in radians.
    %
    %   tau : Column vector (nMotors x 1) containing the virtual joint torques
    %         computed by the controller (e.g., VMC).
    %
    % -------------------------------------------------------------------------
    % Control Law:
    %   The torque command is converted into a position increment using:
    %
    %       Δq = tau / K_theta
    %
    %   where K_theta is a proportional gain linking torque to angular motion.
    %   The resulting joint position is:
    %
    %       q_final = q_initial + Δq
    %
    % -------------------------------------------------------------------------
    % Saturation:
    %   Joint positions are saturated to respect mechanical limits:
    %
    %       q ∈ [−150°, +150°]
    %
    % -------------------------------------------------------------------------
    % Encoder Mapping:
    %   The final joint angle is mapped to Dynamixel encoder units assuming:
    %
    %       - Mechanical range : 300°
    %       - Encoder range    : [0, 1023]
    %       - Zero centered at 150°
    %
    %   Mapping:
    %       raw = ((q + 150°) / 300°) × 1023
    %
    % -------------------------------------------------------------------------
    % Notes:
    %   - Communication and packet errors are checked for each motor.
    %   - Error messages are printed to the MATLAB console when detected.
    %   - The function does not stop execution upon errors.
    %   - This method assumes position control mode on the Dynamixel motors.
    %
    % -------------------------------------------------------------------------
    % Example:
    %   writeMotors(port_num, 1.0, [1 2 3], 30, q, tau);
    %
    % -------------------------------------------------------------------------

    COMM_SUCCESS                = 0;            % Communication Success result value
    
    nMotors = numel(DXL_IDS);
    K_theta = 2;          % Gain torque/angle
    q_min = -deg2rad(150);
    q_max = deg2rad(150);

    for k = 1:nMotors
        id    = DXL_IDS(k);
        dq = tau(k) / K_theta;
        q_final = max(q_min, min(q_max, q_initial(k) + dq));
        q_final = round(((q_final + deg2rad(150)) / deg2rad(300)) * 1023);
        q_final = max(0, min(1023, q_final));

        % Writes the positions in the motors
        write2ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, q_final);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        % Checking communication errors 
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error       = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= 0
            fprintf('[writeSensors] Communication error with motor ID  %d: %s\n', ...
                id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('[writeSensors] Packet error with motor ID %d: %s\n', ...
                id, getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
    end
end