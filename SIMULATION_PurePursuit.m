clc;
clear all;
close all;

% Robot parameters
robotPose = [0,0.2,0];
tireDiameter_m = 0.25;
trackWidth_m = 0.5;
model = 'icr'; % or 'linear'
v_mps = 0.5;

% time between iterations in seconds
dt_s = 0.1;
lookaheaddist_m = 0.25;
goalPoints = [0,0; 2,0; 2,2; 1,1; 2,0];

ii = 1;
lastGoal = robotPose(1:2);
currentGoal = goalPoints(ii, :);
atFinalGoal = 0;
while(~atFinalGoal)
    [w_radps, gp] = purePursuit(robotPose, lastGoal, currentGoal, 0.5);

    % Update robot pose using kinematic model
    robotPose = differentialDriveKinematics(robotPose, v_mps, w_radps, dt_s, model);
    
    % Update goal points if target goal reached
    if(atGoalPoint(robotPose, currentGoal))
        if(ii < length(goalPoints))  
            % increment goal point
            lastGoal = currentGoal;
            ii = ii + 1;
            currentGoal = goalPoints(ii, :);
        else
            % No more goal points in list; at final goal
            atFinalGoal = 1;
        end
    end
    
    % Render environment
    %======================================================================
    clf;
    hold on;
    title(sprintf('Current Goal Point: %0.2f, %0.2f', currentGoal(1), currentGoal(2)))
    xlim([-0.5 3.5]); ylim([-1.5 1.5]);
    xlabel('meters'); ylabel('meters');
    if(exist('robotPose'))
        drawRobot(robotPose(1), robotPose(2), rad2deg(robotPose(3)), 0.25);
        scatter(gp(1), gp(2), 'o');
    end
    pause(0.001);
    % End render environment
    %----------------------------------------------------------------------
end

function [atGoal] = atGoalPoint(robotPose, goalPoint)
    goalRadius_m = 0.25;
    dist = sqrt((robotPose(1) - goalPoint(1))^2 + (robotPose(2) - goalPoint(2))^2);
    atGoal = abs(dist) < goalRadius_m;
end