function [w_radps] = purePursuit(currentPose, lineSegBeginPoint, lineSegEndPoint, v_mps)
%PUREPURSUIT Summary of this function goes here
%   Detailed explanation goes here

    lookaheadDist_m = 0.25;
    
    err = getCrossTrackError(lineSegBeginPoint, lineSegEndPoint, currentPose);
    
    gp = getGoalPoint(currentPose, lineSegBeginPoint, lineSegEndPoint, lookaheadDist_m);
    
    a = currentPose(1:2); b = gp;
    alpha = acos(min(1,max(-1, a(:).' * b(:) / norm(a) / norm(b) )));
    
    w_radps = 0.5;
    
end

function [point] = getGoalPoint(currentPose, lineSegBeginPoint, lineSegEndPoint, r)
    % https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899
    d = lineSegEndPoint - lineSegBeginPoint;
    f = lineSegBeginPoint - currentPose(1:2);
    
    a = dot(d,d);
    b = 2*dot(f,d);
    c = dot(f,f) - r*r;
    discriminant = b*b - 4*a*c;
    
    if (discriminant < 0)
        % no intersection
        point = [];
    else
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant)/(2*a);
        t2 = (-b + discriminant)/(2*a);
        if (t1 >= 0 && t1 <=1)
            % return t1 intersection
            point = lineSegBeginPoint + t1*d;
        elseif (t2 >= 0 && t2 <=1)
            % return t2 intersection
            point = lineSegBeginPoint + t2*d;;
        else
            % otherwise, no intersection
            point = [];
        end
    end

end

function [error] = getCrossTrackError(s1, s2, pose)
    p = projectOntoSegment(s1, s2, pose(1:2));
    
    error = sqrt((p(1) - pose(1))^2 + (p(2) - pose(2))^2);
end

function [projected_p] = projectOntoSegment(s1, s2, p1)
    % get vector that describes line segment
    s = s2 - s1;
    
    % get vector that describes p relative to line segment
    p = p1 - s1;
    
    % project p onto s
    projected_p = (dot(p,s)/dot(s,s)) * s;
    
    % add projected vector onto start of line segment
    projected_p = projected_p + s1;
end

