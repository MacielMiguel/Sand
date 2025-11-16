%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main function from SAND project
%
% No inputs and no outputs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main()
    % System's configuration
    config = loadConfig();

    % Initializes the system (hardware/simulation)
    hw = initSystem(config);

    % Control loop
    t   = 0;
    dt  = config.control.Ts;

    fprintf('Starting SAND control loop...\n');

    while t < config.control.t_final
        % Reading motors states
        state = readSensors(hw, config);
        
        % VMC commands
        u     = vmcController(state, ref, config);

        % Writting VMC commands
        writeActuators(hw, u, config);

        pause(dt);        
        t = t + dt;
    end

    % Plotting the data
    plotData(log, config);

    fprintf('Control loop finished.\n');
end
