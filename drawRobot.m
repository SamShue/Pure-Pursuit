function drawRobot(x, y, theta, radius, color)

    theta = theta - 90;
    % Get points for circle
    angles = 0:5:360;
    x_c = [cosd(angles)',sind(angles)'];
    x_c = x_c.*radius;

    % Get points for orientation arrow
    p1 = [0.2,-0.4];
    p2 = [0.2,0.4];
    p3 = [0.6,0.4];
    p4 = [0,1];
    inv_x = [-1,1];
    x_a = [p1;p2;p3;p4;inv_x.*p3;inv_x.*p2;inv_x.*p1;p1];
    x_a = x_a.*radius;

    % Rotation matrix for theta
    rot_trans = [cosd(theta), -sind(theta), x; sind(theta), cosd(theta), y; 0 0 1];
    
    % Apply rotation and translation to points
    x_c = rot_trans*[x_c,ones(size(x_c(:,1)))]';
    x_a = rot_trans*[x_a,ones(size(x_a(:,1)))]';
    x_c = x_c(1:2,:)';
    x_a = x_a(1:2,:)';
    
    if(nargin < 5)
        plot(x_c(:,1),x_c(:,2),'blue'); hold on;
        plot(x_a(:,1),x_a(:,2),'red');
    else
        if(strcmp(color ,'gray'))
            plot(x_c(:,1),x_c(:,2),'Color',[.7 .7 .7]); hold on;
            plot(x_a(:,1),x_a(:,2),'Color',[.7 .7 .7]);
        else
            plot(x_c(:,1),x_c(:,2),color); hold on;
            plot(x_a(:,1),x_a(:,2),color);
        end
    end
end