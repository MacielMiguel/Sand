function robot = robotParams()
% Physical and geometrical parameters of the robot.

    robot.nLegs          = 4;
    robot.nJointsPerLeg  = 3;
    robot.numJointsTotal = robot.nLegs * robot.nJointsPerLeg;

    % Lengths (m)
    robot.linkLength = [0.10, 0.10, 0.15];

    % Mass (kg)
    robot.massTotal = 3.0;

    % TO COMPLETE
end
