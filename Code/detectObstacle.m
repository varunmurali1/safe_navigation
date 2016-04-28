function [w, obs, obsA, closest] = detectObstacle(w, laserrange, me)

    %% Simple ray tracing
    ang = (me(3) - (pi/4)):0.1: (me(3) + (pi/4));
    dx = [];
    xp = [];
    dy = [];
    yp = [];
    obs = [];
    for i = 1:0.1:laserrange
        dx = [dx; me(1) + i*cos(ang)];
        xp = [xp, me(1) + i*cos(ang)];
        dy = [dy; me(2) + i*sin(ang)];
        yp = [yp, me(2) + i*sin(ang)];
    end
    size_x = w.GridSize(1);
    size_y = w.GridSize(2);
    dx(dx<1) = 1;
    dy(dy<1) = 1;
    dx(dx>size_x) = size_x;
    dy(dy>size_y) = size_y;
    obsA     = [];
    closest(1) = 5;
    closest(2) = 5;
    mindist = 5;
%     size(dx)
%     size(dy)
%     dx
%     dy

    for i = 1:size(dx,2)
        for j = 1:size(dx,1)
            if getOccupancy(w, [dx(j,i) dy(j,i)]) == 1
                obs     = [obs; floor(dx(j,i)), floor(dy(j,i))];
                d       = sqrt( ( obs(end,1) - me(1) ) ^ 2 + (obs(end,2) - me(2)) ^ 2 );
                if d < mindist
                    mindist = d;
                    closest(1) = obs(end,1);
                    closest(2) = obs(end,2);
                    obsA    = ang(i);
                end
                break;
            end
        end
    end
    
    xp(xp<1) = 1;
    yp(yp<1) = 1;
    
end
