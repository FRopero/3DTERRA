function [ Tp ] = scene_generator(problem_params, map_data, cfgParams, itdir)
% This function creates a map ensuring that 'n' target points are distributed 
% in groups of 'delta' groups inside an specific radius 2x'r', around a map 
% with an specific area. 

% INPUTS
% problem_params = ECU_CSURP Parameters
% map_data       = Map data info
% itdir          = save directory of current iteration

% OUTPUTS
% Tp = (X,Y,Z) coordinates of the target points

disp('Generating random scenario ...');

max_factor_sigma = 1;
min_factor_sigma = 0.5;

groups = [];
Tp = [];
print = true;
if (problem_params.N>0)
    if (problem_params.Area>0)
        if (problem_params.D>0)
            delta = problem_params.D;
            n_group = delta;
            rng('shuffle','twister');
            while (delta > 0)
                t = [];
                is_g = false;
                while (isempty(t) || ~is_g)
                    g = [(problem_params.Area*rand(1,1));(problem_params.Area*rand(1,1))];    
                    a = (max_factor_sigma-min_factor_sigma)*rand(1,1)+min_factor_sigma;
                    sigma = a*problem_params.R;
                    t = int64(g + normrnd(0,sigma,[2,problem_params.N/n_group]));
                    [is_g,t] = isGreen(t,map_data);
                end
                Tp = [Tp  t];
                groups = [groups g];
                delta = delta - 1;
            end
        else
            disp('delta must be > 0')
        end
    else
       disp('area must be > 0') 
    end
else
   disp('n must be > 0') 
end

%Round to 3 decimals
f = 10.^3;
Tp = double(round(f*Tp)/f);

if (cfgParams.printResults)
    vis = 'on';
else
    vis = 'off';
end

if (cfgParams.saveResults)
    theta = linspace(0,2*pi);
    %Draw
    fig = figure('visible',vis);
    plot(Tp(1,:),Tp(2,:),'blue.',problem_params.Home(1,1),problem_params.Home(2,1),'*');
    hold on;
    [~,c] = size(groups);
    for i=1:c
        c_x(i,:) = problem_params.R*sin(theta) + groups(1,i);
        c_y(i,:) = problem_params.R*cos(theta) + groups(2,i);
        plot(groups(1,i),groups(2,i),'black+',c_x(i,:),c_y(i,:),'r:');
    end
    hold off;
    title('Random map generated');
    axis equal;
    
    path = [itdir 'scenario.png'];
    saveas(fig,path);
end


end

function [g,t] = isGreen(t,map_data)
    [xmap,ymap] = size(map_data.z_map);
    [~,c] = size(t);
    g = c>0;
    for i=1:c
        if (t(1,i)<=0 || t(2,i)<=0 || t(1,i)>=xmap || t(2,i)>=ymap) 
            g = false;
            break;
        end
        z = map_data.z_map(int64(t(1,i)),int64(t(2,i)));
        if (int64(z) == int64(map_data.map_conf.VALID_MINIMUM/2 - 1))
            g = false;
            break;
        end
        if (z ~=0)
            t(3,i) = z;
        else
            g = false;
            break;
        end
    end
end
