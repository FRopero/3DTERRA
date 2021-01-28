function [ V1 ] = Voronoi3D_V2( T, Home, R, gmap)
%  function [ V ] = Voronoi3D( T, Home, R, map, gmap)
%  This function do the voronoi tessellations to find the minimal set of
%  vertices/charging stops covering the whole set of T.
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    Home(x;y;z) - home location of the multiple UGV-UAV system in meters
%    R    - Radius defining the maximum distance the UAV can travel in meters
%    gmap - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    V1    - set of vertices in meters

% theta = linspace(0,2*pi);
% figure;
% hold on


%Algorithm variables
legit_Vertices = [];
unlegit_Vertices = [];
artificial_Vertices = Home;
nearest_Vertices = [];

V1 = [];
wp_nc = T;
wp_c = [];

[xmap,ymap] = size(gmap);
is_solution = true;

while ( ~isempty(wp_nc)  && is_solution)
    
    if (~isempty(legit_Vertices))
        
        %Search and catch the legit and unlegit vertice to each wp_nc and add to
        %nearest_Vertices
        [~,c_wp] = size(wp_nc);
        [~,c_v] = size(legit_Vertices);
        [~,c_uv] = size(unlegit_Vertices);
        nearest_Vertices = [];
        for i=1:c_wp
            %1. Get the legit_vertices inside the wp(i) radius
            dist_array = [];
            for j=1:c_v
                dist_array = [dist_array sqrt( ((wp_nc(1,i)-legit_Vertices(1,j))^2) + ((wp_nc(2,i)-legit_Vertices(2,j))^2) + ((wp_nc(3,i)-legit_Vertices(3,j))^2))];
            end
            if(~isempty(dist_array))
                [~,pos] = min(dist_array);
                nearest_Vertices = [nearest_Vertices legit_Vertices(:,pos)];
                insideR_Vertices = [];
                for p=1:length(dist_array)
                    if int64(dist_array)<R
                        insideR_Vertices = [insideR_Vertices legit_Vertices(:,p)];
                    end
                end
                if (~isempty(insideR_Vertices))
                    nearest_Vertices = [nearest_Vertices insideR_Vertices];
                else
                    Vgz = double(green_zones_ordered_search(wp_nc(:,i),R,gmap));
                    if (isempty(Vgz))
                        is_solution = false;
                    end
                    nearest_Vertices = [nearest_Vertices Vgz];
                    artificial_Vertices = [artificial_Vertices Vgz];
                end
            end
            %2. Get the unlegit_vertices inside the wp(i) radius
            dist_array = [];
            for j=1:c_uv
                dist_array = [dist_array sqrt( ((wp_nc(1,i)-unlegit_Vertices(1,j))^2) + ((wp_nc(2,i)-unlegit_Vertices(2,j))^2))];
            end
            if(~isempty(dist_array))
                [~,pos] = min(dist_array);
                nearest_Vertices = [nearest_Vertices unlegit_Vertices(:,pos)];
                insideR_Vertices = [];
                for p=1:length(dist_array)
                    if int64(dist_array)<R
                        insideR_Vertices = [insideR_Vertices unlegit_Vertices(:,p)];
                    end
                end
                if (~isempty(insideR_Vertices))
                    nearest_Vertices = [nearest_Vertices insideR_Vertices];
                else
                    Vgz = double(green_zones_ordered_search(wp_nc(:,i),R,gmap));
                    if (isempty(Vgz))
                        is_solution = false;
                    end
                    nearest_Vertices = [nearest_Vertices Vgz];
                    artificial_Vertices = [artificial_Vertices Vgz];
                end
            end
        end
        
        %Delete duplicates
        nearest_Vertices = unique(nearest_Vertices.','rows').';
        
    end
    
    %Add these vertices to the next voronoi iteration
    VOR_Vertices = [wp_nc nearest_Vertices];
    
    %Delete duplicates
    VOR_Vertices = unique(VOR_Vertices.','rows').';
    
    if length(VOR_Vertices(1,:))>2
        %2-D Voronoi Tessellation iteration
        [vx_sol,vy_sol] = voronoi(VOR_Vertices(1,:),VOR_Vertices(2,:));
        [~,cx] = size(vx_sol);
        trusted_Vertices = [];
        unlegit_Vertices = [];
        %Catalog the type of every vertice
        for i=1:cx
            if ((0<int64(vx_sol(1,i))) && (int64(vx_sol(1,i))<=xmap) && (0<int64(vy_sol(1,i))) && (int64(vy_sol(1,i))<=ymap))
                vz_sol = gmap(int64(vx_sol(1,i)), int64(vy_sol(1,i)));
                if (vz_sol == 0)
                    unlegit_Vertices = [unlegit_Vertices double([vx_sol(1,i);vy_sol(1,i);vz_sol])];
                else
                    trusted_Vertices = [trusted_Vertices double([vx_sol(1,i);vy_sol(1,i);vz_sol])];
                end
            end
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
%     
%     %%Draw
%     if ~isempty(wp_nc)
%         plot(wp_nc(1,:),wp_nc(2,:),'.black')
%     end
%     if ~isempty(wp_c) 
%         plot(wp_c(1,:),wp_c(2,:),'.blue') 
%     end
%     if ~isempty(legit_Vertices)
%         plot(legit_Vertices(1,:),legit_Vertices(2,:),'+green')
%     end
%     if ~isempty(unlegit_Vertices)
%         plot(unlegit_Vertices(1,:),unlegit_Vertices(2,:),'+red')
%     end
%     [~,c]= size(V1);
%     for i=1:c
%         c_x(i,:) = R*cos(theta) + V1(1,i);
%         c_y(i,:) = R*sin(theta) + V1(2,i);
%         plot(c_x(i,:),c_y(i,:),'r:');
%     end
%     drawnow;
%     axis equal;
%     pause(1)
    
end

if (~is_solution)
    V1 = [];
else
    V1 = unique(V1.','rows').';
end

end

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



