function [ V1 ] = Voronoi3D( T, Home, R, dtm_map, gmap)
%  function [ V ] = Voronoi3D( T, Home, R, map, gmap)
%  This function do the voronoi tessellations to find the minimal set of
%  vertices/charging stops covering the whole set of T.
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    Home(x;y;z) - home location of the multiple UGV-UAV system in meters
%    R    - Radius defining the maximum distance the UAV can travel in meters
%    dtm_map  - Matrix with the map points (x,y,z) in meters.
%    gmap - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    V1    - set of vertices in meters

%Algorithm variables
V1 = [];
wp_c = [];
Vx = [];
Vy = [];
Vz = [];
duplicates = 0;
vert_unleg = [];
near_wd = [];
wp_nc = T;
wp_nc_cnt = length(wp_nc(1,:));
art_vert = Home;
v_used = [];
[xmap,ymap] = size(gmap);
is_solution = true;

while ( 0 < wp_nc_cnt  && is_solution)
    
    % [STEP 1] - Calculate the Voronoi's Diagram over the WPm not covered
    
    if (~isempty(Vx)) %For n iterations
        
        %Search and catch the nearest vertice to each wp_nc and add to
        %near_v
        [~,c_wp] = size(wp_nc);
        [~,c_vx] = size(Vx);
        near_v = [];
        t = 1;
        dminT = Inf;
        for i=1:c_wp
            dmin = sqrt( ((wp_nc(1,i)-Vx(1))^2) + ((wp_nc(2,i)-Vy(1))^2) + ((wp_nc(3,i)-Vz(1))^2));
            near_v(:,i) = [Vx(1);Vy(1);Vz(1)];
            for j=1:c_vx
                d = sqrt( ((wp_nc(1,i)-Vx(j))^2) + ((wp_nc(2,i)-Vy(j))^2) + ((wp_nc(3,i)-Vz(j))^2));
                if (d < dmin)
                    dmin = d;
                    near_v(:,t) = [Vx(j);Vy(j);Vz(j)];
                    t = t + 1;
                end
            end
            if (dmin < dminT)
                dminT = dmin;
            end
        end
        
        %Delete duplicates in near_v and add them to near_wd
        near_wd = delete_Duplicates(near_v);
        
        %Count duplicates and so, infinite loop in voronoi diagram
        [~,lv] = size(near_wd);
        [~,lu] = size(v_used);
        duplicates = 0;
        for i=1:lv
            for j=1:lu
                if (near_wd(:,i)==v_used(:,j))
                    duplicates = duplicates + 1;
                end
            end
        end
        v_used = [v_used near_wd];
    end
    
    %Add these vertices to the next voronoi iteration
    wp_plus_v = [wp_nc near_wd vert_unleg];
    
    %We can't do voronoi if we have 2 points or less (one wp and its nearest vertice)
    %or Only 2 wp and 1 vértice, and the distance between the WPs and their common nearest
    %vertice, is less than the R. This situation cause an infinite loop.
    if (length(wp_plus_v(1,:))<=2 || duplicates > 0)
        %Catch the first uncovered target point and search for the nearest
        %first vertice
        Vgz = green_zones_search(wp_plus_v(:,1),R,dtm_map,gmap);
        %Vgz = green_zones_ordered_search(wp_plus_v(:,1),R,gmap);
        %If it is empty there is no solution because there is no vertice inside
        %a green zone in radius for this waypoint
        if (isempty(Vgz))
            is_solution = false;
        end
        art_vert = [art_vert Vgz];
    
    else
        %2-D Voronoi Tessellation iteration
        [vx_s,vy_s] = voronoi(wp_plus_v(1,:),wp_plus_v(2,:));
        [~,cx] = size(vx_s);
        vz_s = [];
        %Catalog the type of every vertice
        vert_unleg = [];
        for i=1:cx
            if ((0<int32(vx_s(1,i))) && (int32(vx_s(1,i))<=xmap) && (0<int32(vy_s(1,i))) && (int32(vy_s(1,i))<=ymap))
                vz_s(1,i) = gmap(int32(vx_s(1,i)), int32(vy_s(1,i)));
                if (vz_s(1,i) == 0)
                    vz_s(1,i) = -1;
                    vert_unleg = [vert_unleg double([int32(vx_s(1,i));int32(vy_s(1,i));0])];
                    vert_unleg = delete_Duplicates(vert_unleg);
                end
            else
                vz_s(1,i) = -1;
            end
        end
        
        %Delete the duplicates vertices and out of boundaries vertices
        Vp = delete_Duplicates([vx_s(1,:); vy_s(1,:); vz_s]);
        if (~isempty(Vp)) 
            Vx = Vp(1,:);
            Vy = Vp(2,:);
            Vz = Vp(3,:);
        end
    end
    
    %Add artificial vertices created in the last iteration
    Vx = double([art_vert(1,:) Vx]);
    Vy = double([art_vert(2,:) Vy]);
    Vz = double([art_vert(3,:) Vz]);
    
    %Compute the new set of uncovered waypoints
    [r,c] = size(wp_nc);
    
    %Add the best vertices in this iteration to cover to existing target points
    for i=1:c
        dmin = R;
        best_v = -1;
        wp_v = -1;
        [~,t] = size(Vx);
        for j=1:t
            if (wp_nc(1,i)~=-1)
                d = sqrt( ((wp_nc(1,i)-Vx(j))^2) + ((wp_nc(2,i)-Vy(j))^2) + ((wp_nc(3,i)-Vz(j))^2));
                if (d < dmin && best_v ~= 1) %%Home=1 is the best option
                    dmin = d;
                    best_v = j;
                    wp_v = i;
                end
            end
        end
        if (best_v~=-1)
            wp_c = [wp_c wp_nc(:,wp_v)];
            wp_nc(:,wp_v) = -1;
            wp_nc_cnt = wp_nc_cnt - 1;
            V1 = [V1 [Vx(1,best_v);Vy(1,best_v);Vz(1,best_v)]];
        end
    end
    
    [~,c] = size(wp_nc);
    del = 0; %To count the number of wp covered this iteration
    for i=1:c
        if (wp_nc(1,i)==-1)
            del = del + 1;
        end
    end
    
    %Update wp_nc Table for the next iteration
    if (del>0)
        tmp_wp_nc = wp_nc;
        wp_nc = zeros(r,c-del);
        for i=1:r
            t_y = 0;
            for j=1:c
                if (tmp_wp_nc(i,j)~=-1)
                    t_y = t_y + 1;
                    wp_nc(i,t_y) = tmp_wp_nc(i,j);
                end
            end
        end
    end
    
end


if (~is_solution)
    V1 = [];
end

end

function [v_sol] = delete_Duplicates(vertices)
%Delete duplicates vertices
v_sol = [];
[~,c] = size(vertices);
for i=1:c
    v = vertices(:,i);
    in = 0;
    [~,c] = size(v_sol);
    if (v(3,1)~=-1)
        for j=1:c
            if (v==v_sol(:,j))
                in = in + 1;
                break
            end
        end
        if in==0
            v_sol = [v_sol v];
        end
    end
end
end



