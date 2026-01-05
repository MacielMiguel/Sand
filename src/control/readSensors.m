function q = readSensors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_PRESENT_POSITION)
    % readSensors
    % -------------------------------------------------------------------------
    % Reads the current angular position of multiple Dynamixel motors and
    % converts the raw encoder values into joint angles in radians.
    %
    % This function queries the "Present Position" register of each motor using
    % the Dynamixel SDK, applies a linear mapping from encoder ticks to angular
    % position, and returns the result as a column vector.
    %
    % -------------------------------------------------------------------------
    % Syntax:
    %   q = readSensors(port_num, PROTOCOL_VERSION, DXL_IDS, ADDR_PRESENT_POSITION)
    %
    % -------------------------------------------------------------------------
    % Inputs:
    %   port_num  : Port handler returned by portHandler() after opening
    %               the serial communication.
    %
    %   PROTOCOL_VERSION : Dynamixel communication protocol version
    %                      (e.g., 1.0 for AX-series, 2.0 for X-series).
    %
    %   DXL_IDS   : Row or column vector containing the IDs of the motors
    %               to be read (e.g., [1 2 3]).
    %
    %   ADDR_PRESENT_POSITION : Address of the "Present Position" register
    %                           in the Dynamixel control table
    %                           (e.g., 36 for AX-12A, Protocol 1.0).
    %
    % -------------------------------------------------------------------------
    % Outputs:
    %   q : Column vector (nMotors x 1) containing the joint positions
    %       expressed in radians.
    %
    %       The conversion assumes:
    %         - Encoder range: [0, 1023]
    %         - Mechanical range: 300 degrees
    %         - Zero position centered at 150 degrees
    %
    %       Mapping:
    %         q = (raw / 1023) * 300° − 150°, converted to radians
    %
    % -------------------------------------------------------------------------
    % Notes:
    %   - Communication and packet errors are checked for each motor.
    %   - Error messages are printed to the MATLAB console when detected.
    %   - No exception is thrown; the function continues reading remaining motors.
    %
    % -------------------------------------------------------------------------
    % Example:
    %   q = readSensors(port_num, 1.0, [1 2 3], 36);
    %
    % -------------------------------------------------------------------------

    nMotors = numel(DXL_IDS);
    q = zeros(nMotors,1);

    for k = 1:nMotors
        id = DXL_IDS(k);

        % Reading position values in the interval [0, 1023]
        q(k) = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
        q(k) = (double(q(k)) / 1023) * deg2rad(300) - deg2rad(150);
        
        % Checking communication erros
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error       = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= 0
            fprintf('[readSensors] Error with motor ID %d: %s\n', ...
                id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('[readSensors] Error with motor ID %d: %s\n', ...
                id, getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
    end
end
