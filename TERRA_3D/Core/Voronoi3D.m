%  function [ V ] = Voronoi3D( T, Home, R, gmap)
%  It computes Voronoi Tessellations to find the minimal set of
%  vertices/charging stops covering the whole set of Target Points (T).
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    Home(x;y;z) - home location of the multiple UGV-UAV system in meters
%    R           - Radius defining the maximum distance the UAV can travel in meters
%    gmap        - Matrix with the green zones defined. X=green zone 0=Otherwise
%    cfg_params  - testing configuration parameters
%  Outputs:
%    V1    - set of vertices in meters
%    figV  - solution figures

function [ V1, figV ] = Voronoi3D( T, Home, R, gmap, cfg_params)

figV = [];
if (cfg_params.printResults)
    vis = 'on';
else
    vis = 'off';
end

%First iteration plot
figInit = figure('visible',vis);
figInit.Name = 'Voronoi_FirstIteration';
if (cfg_params.saveResults)
    figV = [figV figInit];
end
[vx,vy] = voronoi(T(1,:),T(2,:));
hold on
plot(T(1,:),T(2,:),'blue.',vx,vy,'black-',vx,vy,'green+');
[~,c]= size(vx);
theta = linspace(0,2*pi);
for i=1:c
    c_x(i,:) = R*cos(theta) + vx(i);
    c_y(i,:) = R*sin(theta) + vy(i);
    plot(c_x(i,:),c_y(i,:),'r:');
end
hold off
axis equal
title('First Voronoi Iteration');

%figIt = figure('Visible','off');
%figIt.Name = 'Voronoi_All';
%if (cfg_params.saveResults)
%    figV = [figV];
%end
%hold on

%Algorithm variables
legit_Vertices = [];
unlegit_Vertices = [];

if gmap(int64(Home(1,1)), int64(Home(2,1)) ~= 0)
    artificial_Vertices = Home;
else
    artificial_Vertices = [];
end
used_Vertices = [];

V1 = [];
wp_c = [];
wp_nc = T;

[xmap,ymap] = size(gmap);
is_solution = true;

while ( ~isempty(wp_nc)  && is_solution)
    
    nearest_Vertices = [];
    [~,c_wp] = size(wp_nc);
    
    %1. Get the legit_vertices inside the wp(i) radius
    if (~isempty(legit_Vertices))
        [~,c_v] = size(legit_Vertices);
        for i=1:c_wp
            dist_array = [];
            for j=1:c_v
                dist_array = [dist_array sqrt( ((wp_nc(1,i)-legit_Vertices(1,j))^2) + ((wp_nc(2,i)-legit_Vertices(2,j))^2) + ((wp_nc(3,i)-legit_Vertices(3,j))^2))];
            end
            if(~isempty(dist_array))
                [~,pos] = min(dist_array);
                nearest_Vertices = [nearest_Vertices legit_Vertices(:,pos)];
                for p=1:length(dist_array)
                    if int64(dist_array)<R
                        nearest_Vertices = [nearest_Vertices legit_Vertices(:,p)];
                    end
                end
            end
        end 
    end
    
    %2. Get the unlegit_vertices inside the wp(i) radius
    if (~isempty(unlegit_Vertices))
        [~,c_uv] = size(unlegit_Vertices);
        for i=1:c_wp
            dist_array = [];
            for j=1:c_uv
                dist_array = [dist_array sqrt( ((wp_nc(1,i)-unlegit_Vertices(1,j))^2) + ((wp_nc(2,i)-unlegit_Vertices(2,j))^2))];
            end
            if(~isempty(dist_array))
                [~,pos] = min(dist_array);
                nearest_Vertices = [nearest_Vertices unlegit_Vertices(:,pos)];
                for p=1:length(dist_array)
                    if int64(dist_array)<R
                        nearest_Vertices = [nearest_Vertices unlegit_Vertices(:,p)];
                    end
                end
            end
        end
    end
    
    %Delete duplicates
    nearest_Vertices = unique(nearest_Vertices.','rows').';  
    
    %Add these vertices to the next voronoi iteration
    VOR_Vertices = [wp_nc nearest_Vertices];
    
    %We can't do voronoi if we have 2 points or less (one wp and its nearest vertice)
    %or Only 2 wp and 1 vértice, and the distance between the WPs and their common nearest
    %vertice, is less than the R. This situation cause an infinite loop.
    if (length(VOR_Vertices(1,:))<=2 || isDuplicates(nearest_Vertices,used_Vertices))
        %Catch the first uncovered target point and search for the nearest
        %first vertice
        [~,c_wp] = size(wp_nc);
        for i=1:c_wp
            Vgz = green_zones_ordered_search(VOR_Vertices(:,i),R,gmap);
            %If it is empty there is no solution because there is no vertice inside
            %a green zone in radius for this waypoint
            if (isempty(Vgz))
                is_solution = false;
            end
            artificial_Vertices = [artificial_Vertices double(Vgz)];
            VOR_Vertices = [VOR_Vertices double(Vgz)];
        end
    end
    
    used_Vertices = [used_Vertices nearest_Vertices];
    
    %Delete duplicates
    VOR_Vertices = unique(VOR_Vertices.','rows').';
    
    if length(VOR_Vertices(1,:))>2
        %2-D Voronoi Tessellation iteration
        [vx_sol,vy_sol] = voronoi(VOR_Vertices(1,:),VOR_Vertices(2,:));
        
        [~,cx] = size(vx_sol);
        trusted_Vertices = [];
        unlegit_Vertices = [];
        outbounds_Vertices = [];
        %Catalog the type of every vertice
        for i=1:cx
            if ((0<int64(vx_sol(1,i))) && (int64(vx_sol(1,i))<=xmap) && (0<int64(vy_sol(1,i))) && (int64(vy_sol(1,i))<=ymap))
                vz_sol = gmap(int64(vx_sol(1,i)), int64(vy_sol(1,i)));
                if (vz_sol == 0)
                    unlegit_Vertices = [unlegit_Vertices double([int64(vx_sol(1,i));int64(vy_sol(1,i));int64(vz_sol)])];
                else
                    trusted_Vertices = [trusted_Vertices double([int64(vx_sol(1,i));int64(vy_sol(1,i));int64(vz_sol)])];
                end
            else
                outbounds_Vertices = [outbounds_Vertices double([int64(vx_sol(1,i)/2);int64(vy_sol(1,i)/2);0])];
            end
        end
        
        if (isempty(unlegit_Vertices) && isempty(trusted_Vertices))
            unlegit_Vertices = outbounds_Vertices;
        end

        %Delete Duplicates
        trusted_Vertices = unique(trusted_Vertices.','rows').';

        %Delete the duplicates vertices and out of boundaries vertices
        legit_Vertices = trusted_Vertices;
    end
    
    %Add artificial vertices created in the last iteration
    legit_Vertices = double([artificial_Vertices legit_Vertices]);
        
    %Compute the new set of uncovered waypoints
    [~,c] = size(wp_nc);
    del_vertices = [];
    %Add the best vertices in this iteration to cover to existing target points
    for i=1:c
        dmin = R;
        best_v = -1;
        wp_v = -1;
        [~,t] = size(legit_Vertices);
        for j=1:t
            d = sqrt( ((wp_nc(1,i)-legit_Vertices(1,j))^2) + ((wp_nc(2,i)-legit_Vertices(2,j))^2) + ((wp_nc(3,i)-legit_Vertices(3,j))^2));
            if (d < dmin && best_v ~= 1) %%Home=1 is the best option
                dmin = d;
                best_v = j;
                wp_v = i;
            end
        end
        if (best_v~=-1)
            %New covered target point
            wp_c = [wp_c wp_nc(:,wp_v)];
            del_vertices = [del_vertices wp_v];
            
            %Found new vertice for the solution
            V1 = [V1 legit_Vertices(:,best_v)];
        end
    end
    
    %Update wp_nc table
    [~,d] = size(del_vertices);
    if (d>0)
        wp_nc(:,del_vertices(1:d)) = [];
    end
    
    %Draw Method
    %drawIteration(wp_nc,wp_c,legit_Vertices,unlegit_Vertices,V1,R,figIt);
    
end
    %figure(figIt);
    %figIt.Visible = 'off';
    %hold off;
    %axis equal;
    %title('2D Voronoi Iterations');
    
if (~is_solution)
    V1 = [];
else
    V1 = unique(V1.','rows').';
    
    %Last iteration plot
    figLast = figure('visible',vis);
    figLast.Name = 'Voronoi_Solution';
    if (cfg_params.saveResults)
        figV = [figV figLast];
    end
    hold on
    plot(T(1,:),T(2,:),'blue.',V1(1,:),V1(2,:),'green+');
    [~,c]= size(V1);
    theta = linspace(0,2*pi);
    for i=1:c
        c_x(i,:) = R*cos(theta) + V1(1,i);
        c_y(i,:) = R*sin(theta) + V1(2,i);
        plot(c_x(i,:),c_y(i,:),'r:');
    end
    hold off
    axis equal
    title('Voronoi Solution');
end

end

% Search duplicates between two vectors
function [bool] = isDuplicates(lA,lB)
bool = false;
%Count duplicates and so, infinite loop in voronoi diagram
[~,lv] = size(lA);
[~,lu] = size(lB);

for i=1:lv
    for j=1:lu
        if (lA(:,i)==lB(:,j))
            bool = true;
            break
        end
    end
end
end

% Draw function
function [] = drawIteration(wp_nc,wp_c,legit_Vertices,unlegit_Vertices,V1,R,f)
f.Visible = 'off';
figure(f);

if ~isempty(wp_nc)
    plot(wp_nc(1,:),wp_nc(2,:),'.black')
end
if ~isempty(wp_c) 
    plot(wp_c(1,:),wp_c(2,:),'.blue') 
end
if ~isempty(legit_Vertices)
    plot(legit_Vertices(1,:),legit_Vertices(2,:),'+green')
end
if ~isempty(unlegit_Vertices)
    plot(unlegit_Vertices(1,:),unlegit_Vertices(2,:),'+red')
end
[~,c]= size(V1);
theta = linspace(0,2*pi);
for i=1:c
    c_x(i,:) = R*cos(theta) + V1(1,i);
    c_y(i,:) = R*sin(theta) + V1(2,i);
    plot(c_x(i,:),c_y(i,:),'r:');
end

%drawnow;
%lgd = legend('Tp not covered (black)','Tp covered (blue)','Legit vertices (green)','Unlegit vertices (red)');
%title(lgd,'');
%legend('boxoff');
%legend('Location','northwestoutside');
%pause(1)
end


