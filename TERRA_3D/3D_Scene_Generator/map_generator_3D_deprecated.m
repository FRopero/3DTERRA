function [ T ] = map_generator_3D(dtm_map, dtm_conf, Home, N, R, delta, tries)
% This function creates a map ensuring that 'n' target points are distributed
% in groups of 'delta' members inside an specific radius 2x'r', around a map
% with an specific area.
% Mandatory: 'n' must be multiple of 'delta'

% INPUTS
% dtm_map  :Matrix with the Mars Map
% dtm_conf :Configuration parameters of the dtm_map
% Home     :Home point
% N        :Number of target points to generate
% R        :Maximum distance the UAV can travel in m
% delta    :Number of target points for vertice or charging stop
% tries    :Number of tries to generate a target point distribution with
% this parameters

% OUTPUTS
% T = coordinates of the target points


groups = [];
T = [];
theta = linspace(0,2*pi);
finish = false;
[xmap,ymap] = size(dtm_map);

xmap = xmap ;
ymap = ymap ;

if (N>0)
    if (delta>0)
        groups_number = N / delta;
        rng('shuffle','twister');
        
        while (groups_number > 0 && ~finish)
            cond = false;
            it = 0;
            %First, ensure that the distance between g and other groups is
            %higher than 4*r
            while (~cond)
                cond = true;
                
                x_p = 0;
                y_p = 0;
                found = false;
                
                while (~found)
                    while (0==x_p) || (0==y_p)
                        x_p = int64(xmap*rand(1,1));
                        y_p = int64(ymap*rand(1,1));
                    end
                    if (int64(dtm_map(x_p,y_p)) ~= int64(dtm_conf.VALID_MINIMUM/2 - 1))
                        found = true;
                    else
                        x_p = int64(xmap*rand(1,1));
                        y_p = int64(ymap*rand(1,1));
                    end
                end

                g = double([x_p;y_p;dtm_map(x_p,y_p)]);
                
                [~,ymax] = size(groups);
                for i=1:ymax
                    d = sqrt( ((groups(1,i)-g(1,1))^2) + ((groups(2,i)-g(2,1))^2) + ((groups(3,i)-g(3,1))^2) );
                    if (d <= 3*R)
                        cond = false;
                    end
                end
                
                d = sqrt( ((Home(1,1)-g(1,1))^2) + ((Home(2,1)-g(2,1))^2) + ((Home(3,1)-g(3,1))^2) );
                if (d <= 3*R)
                    cond = false;
                end
                if (it>100) %Tries per group
                    if (tries>0)
                        [ T ] = map_generator_3D(dtm_map,dtm_conf, Home, N, R, delta, tries-1);
                    end
                    finish = true;
                    break
                end
            end
            if (~finish)
                groups = [groups g];
                %Third, generate 'delta' target points for the group 'g'
                max_x = g(1)+R;
                min_x = g(1)-R;
                max_y = g(2)+R;
                min_y = g(2)-R;
                
                tmp = delta;
                while (tmp > 0)
                    p = [(max_x-min_x)*rand(1,1)+min_x;(max_y-min_y)*rand(1,1)+min_y];
                    if ((0<int64(p(1))) && (int64(p(1))<=xmap) && (0<int64(p(2))) && (int64(p(2))<=ymap))
                        z_p = dtm_map(int64(p(1)),int64(p(2)));
                        if (int64(z_p) ~= int64(dtm_conf.VALID_MINIMUM/2 - 1))
                            d = sqrt( ((g(1,1)-p(1,1))^2) + ((g(2,1)-p(2,1))^2 + ((g(3,1)-z_p)^2)));
                            if (d <= R)
                                p(3,1) = z_p;
                                T = [T p];
                                tmp = tmp - 1;
                            end
                        end
                    end
                    
                end
                
                groups_number = groups_number - 1;
            end
        end
    else
        disp('delta must be > 0')
    end
else
    disp('n must be > 0')
end

%Round to 3 decimals
%f = 10.^3;
%T = round(f*T)/f;

if (~finish)
    %Draw
%     figure;
%     plot(T(1,:),T(2,:),'blue.',Home(1,1),Home(2,1),'*');
%     hold on;
%     [~,ymax] = size(groups);
%     for i=1:ymax
%         c_x(i,:) = R*sin(theta) + groups(1,i);
%         c_y(i,:) = R*cos(theta) + groups(2,i);
%         plot(groups(1,i),groups(2,i),'black+',c_x(i,:),c_y(i,:),'r:');
%     end
%     hold off;
%     axis equal;
%     set(gca,'xDir','reverse')
%     title('Random map generated');
end
end

