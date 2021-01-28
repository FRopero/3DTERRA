% WELCOME TO THE TERRA3D ALGORITHM
%  This function executes the whole solution designed and developed to solve
%  the Energy Constrained UAV and Charging Station UGV Routing Problem (ECU-CSURP).
%
%  The goal is to visit all the waypoints with the UAV, supporting it with
%  UGV as a moving recharging station.

%  - INPUTS -
%  cfg_params     :configuration parameters
%  problem_params :ECU_CSURP Parameters
%  map_data       :3D Map Data
%  dana_params    :parameters for the external UGV path planning application
%  uav_data       :uav parameters

%  - OUTPUTS -
%  data_sol :list of structures with the cost of the paths of each solution.
%   struct {
%      f_ugv_d       :Cost of the UGV as distance travelled in meters.
%      f_ugv_t       :Cost of the UGV as time in seconds.
%      f_uav_d       :Cost of the UAV as distance travelled in meters. (SEARCH_UAV_PATH)
%      f_uav_t       :Cost of the UAV as time in seconds. (SEARCH_UAV_PATH)
%      ftotal_d      :Total cost in meters.
%   }
%  path_sol :paths of both UGV-UAV to reach the solutions computed in the algorithm.
%  figures  :list of figures with the solutions of each algorithm stage
function [data_sol, path_sol, figures] = TERRA3D(cfg_params, problem_params, map_data, dana_params, uav_data)

%Init Output Variables
data_sol = [];
path_sol = [];
figures  = [];

% (1) Voronoi Tessellations Problem
[V1,figV] = Voronoi3D(problem_params.T, problem_params.Home, problem_params.R, map_data.gz_map, cfg_params);

% (2) Hitting Set Problem
[SolL,scp_table,V2,figG] = greedy_scp_3D(problem_params.T, V1, problem_params.R, cfg_params);

% (3) Gravitational Optimization Algorithm
[V3, Vres, figO] = Gravitational_Optimization_3D(problem_params.T, problem_params.Home, V1, problem_params.Gp, SolL, scp_table, problem_params.R, map_data, cfg_params);

% (4) Compute the UGV's path
% (4.1) TSP for the UGV.
[UGV_path] = tsp_ga_ugv3D(V3, dana_params.ugv_tsp);
% (4.2) 3Dana for the TSP vertices
[ ugv_path, ugv_distance] = compute_3Dana( dana_params.map_file, dana_params, UGV_path(:,1:2), map_data.map_type, cfg_params);
if (ugv_path ~= -1)
    % (5) Solve the TSP for the UAV
    [uav_paths, uav_distance, total_stops] = uav_compute_3Dpath(problem_params.T, scp_table, SolL, Vres, ugv_path', uav_data, map_data);
    
    % Collect Results
    [data_sol, path_sol] = collectResults(data_sol, path_sol, ugv_distance, uav_distance,total_stops, ugv_path, uav_paths);
    
    %Save All figures
    figures = [figures figV figG figO printResult(map_data,problem_params,V2,V3,ugv_path,uav_paths,dana_params,cfg_params)];
else
    disp('Whoops! 3Dana was not able to find a route for the UGV. Unlucky :(')
end
end

% Function to collect the results of the execution
function [data_sol, path_sol] = collectResults(data_sol, path_sol, ugv_distance, uav_distance,total_stops, ugv_path, uav_paths)

    total_distance = ugv_distance + uav_distance;
    s = struct('f_ugv',ugv_distance,'f_uav',uav_distance,'f_total',total_distance,'f_stops',total_stops);
    data_sol = [data_sol ; s];
    [ p_sol ] = build_matrix_solution_3D(ugv_path, uav_paths);
    path_sol = [path_sol ; struct('path_solution',p_sol)];
    
end

% Function to print the results of the TERRA3D Algorithm
function fig = printResult(map_data,problem_params,V2,V3,ugv_path,uav_paths,dana_params, cfg_params)

if (cfg_params.printResults)
    vis = 'on';
else
    vis = 'off';
end

fig = figure('Name','final_solution','NumberTitle','off','visible',vis);
h = axes;
surfl(map_data.z_map');
colormap(pink);
shading interp;
hold on

%%% Draw Home %%%
scatter3(problem_params.Home(1,:),problem_params.Home(2,:),problem_params.Home(3,:),'filled','black');
% [x,y,z] = sphere(50);
% hSurface = surf(R*x+Home(2,1),R*y+Home(1,1),R*z+Home(3,1));
% set(hSurface,'FaceColor','green','FaceAlpha',0.4,'FaceLighting','gouraud','EdgeColor','none');

%%% Draw Target Points %%%
scatter3(problem_params.T(1,:),problem_params.T(2,:),problem_params.T(3,:),'filled','blue');
% [r,~] = size(T);
% for j=1:r
%     [x,y,z] = sphere(50);
%     hSurface = surf(R*x+T(j,2),R*y+T(j,1),R*z+T(j,3));
%     set(hSurface,'FaceColor','blue','FaceAlpha',0.25,'FaceLighting','gouraud','EdgeColor','none');
% end

% (1) Voronoi Solution
% [~,c] = size(V1);
% for i=1:c
%   scatter3(V1(2,i),V1(1,i),V1(3,i),'filled','white');
%   [x,y,z] = sphere(50);
%   hSurface = surf(R*x+V1(2,i),R*y+V1(1,i),R*z+V1(3,i));
%   set(hSurface,'FaceColor','white','FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','none');
% end

% (2) HSP Solution
[~,c] = size(V2);
[x,y,z] = sphere(50);
for i=1:c
    if (V2(:,i)==problem_params.Home)
        hSurface = surf(problem_params.R*x+problem_params.Home(1,1),problem_params.R*y+problem_params.Home(2,1),problem_params.R*z+problem_params.Home(3,1));
        %set(hSurface,'FaceColor','green','FaceAlpha',0.4,'FaceLighting','gouraud','EdgeColor','none');
    end
end

%(3) GOA Solution
[~,c] = size(V3);
for i=1:c
    scatter3(V3(1,i),V3(2,i),V3(3,i),'filled','red');
    [x,y,z] = sphere(50);
    hSurface = surf(problem_params.R*x+double(V3(1,i)),problem_params.R*y+double(V3(2,i)),problem_params.R*z+double(V3(3,i)));
    set(hSurface,'FaceColor','green','FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','none');
end

% (4) UGV TSP Solution
if (ugv_path ~= -1)
    z_norm = ((ugv_path(:,3)*2)+map_data.map_conf.VALID_MINIMUM)/2;
    path = plot3(ugv_path(:,1),ugv_path(:,2),z_norm,'g');
    path.LineWidth = 3;
end

% (5) UAV TSP SOlution
if (~isempty(uav_paths))
    [~,c] = size(uav_paths);
    for i=1:c
        path = plot3(uav_paths(i).Coordinates(1,:),uav_paths(i).Coordinates(2,:),uav_paths(i).Coordinates(3,:),':blue');
        path.LineWidth = 3;
    end
end

xlabel('X-Axis')
ylabel('Y-Axis')
zlabel('Z-Axis')

title(dana_params.map_file);
lgd = legend(strcat('Slope = ',num2str(dana_params.slope)),strcat('R = ',num2str(problem_params.R)),'Location','northwest');
title(lgd,'UGV-UAV Parameters');
legend('boxoff')
camlight
hold off
axis equal;

end

% Function to build the results in a single struct
function [ path_sol ] = build_matrix_solution_3D( ugv_path, uav_path )
%%% Build Final Solution Path for the UGV and UAV
path_sol = [];
for i=1:length(ugv_path)
    path_sol = [path_sol struct('c_ugv',ugv_path(i,:),'c_uav',[])];
end

%Insert c_uav to the final path solution
[~,r] = size(path_sol);
[~,u] = size(uav_path);
for i=1:r
    for j=1:u
        if (path_sol(i).c_ugv(1)==uav_path(j).Coordinates(1,1) && path_sol(i).c_ugv(2)==uav_path(j).Coordinates(2,1))
            path_sol(i).c_uav = uav_path(j).Coordinates;
        end
    end
end

end