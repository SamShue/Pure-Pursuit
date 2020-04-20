function [newPose] = differentialDriveKinematics(pose, v_mps, w_radps, dt_s, model)
%DIFFERENTIALDRIVEKINEMATICS Kinematic model of 2D differential drive robot
%   This function performs the kinematics of a 2D differential drive robot
%   based on a specified model. If a model is not specified, the robot will
%   use the linear model. The user may specify 'icr' or 'simple'. The user
%   is required to provide the robot's pose (x_m, y_m, theta_rad), linear
%   velocity (mps), and rotational velocity (radps).

% If no model is specified, use the 'simple' model
if(nargin < 4), model = 'simple'; end

if(strcmp(model, 'icr'))
    % The instantenous center of rotation model. This model projects the
    % robot motion along a circle of radius r, which is determined by the
    % robot's linear and rotational velocity. 
    
    % compute radius of curvature
    r = abs(v_mps/w_radps);
    
    theta = pose(3);
    % compute sin and cos at initial pose
    s = sin(theta);
    c = cos(theta);
    % copute sin and cos at final pose
    s_th = sin(theta + w_radps*dt_s);
    c_th = cos(theta + w_radps*dt_s);

    if(w_radps < 0.05)
        % if the robot is moving straight, then that corresponds to a 
        % circle with an infinite radius. Here we must exclude the r
        % variable to account for that.
        pose(1) = pose(1) + (v_mps*c_th)*dt_s;
        pose(2) = pose(2) + (v_mps*s_th)*dt_s;
        pose(3) = pose(3) + w_radps*dt_s;
    else
        % robot is moving with some curavature
        pose(1) = pose(1) + (-r*s)+(r*s_th);
        pose(2) = pose(2) +( r*c)-(r*c_th);
        pose(3) = pose(3) + w_radps*dt_s;
    end
else
    % The simple model. This model does not account for curvature in motion
    % and applies motion as a linear move then rotation.
    
    % x + v*dt*cos(theta)
    % y + v*dt*sin(theta)
    % theta + dt*w
    
    theta = pose(3);
    % create rotation matrix to get velocities in global frame
    R = [cos(theta) 0;
         sin(theta) 0;
         0          1];
    u = [v_mps; w_radps];
    % update pose with global frame velocities
    pose = pose + R*u.*dt_s;
    
    
    
end

newPose = pose;
end

